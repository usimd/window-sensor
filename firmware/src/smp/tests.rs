extern crate std;

use std::vec;
use std::vec::Vec;

use embedded_storage::nor_flash::{ErrorType, NorFlash, NorFlashErrorKind, ReadNorFlash};
use minicbor::Encoder;
use minicbor::data::Type;

use super::*;
use crate::partition::MCU_BOOT_LAYOUT;

struct FakeFlash(Vec<u8>);

impl FakeFlash {
    fn new() -> Self {
        Self(vec![0xFF; PartitionLayout::FLASH_SIZE as usize])
    }
}

impl ErrorType for FakeFlash {
    type Error = NorFlashErrorKind;
}

impl ReadNorFlash for FakeFlash {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let o = offset as usize;
        bytes.copy_from_slice(
            self.0
                .get(o..o + bytes.len())
                .ok_or(NorFlashErrorKind::OutOfBounds)?,
        );
        Ok(())
    }

    fn capacity(&self) -> usize {
        self.0.len()
    }
}

impl NorFlash for FakeFlash {
    const WRITE_SIZE: usize = 16;
    const ERASE_SIZE: usize = 4096;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        assert_eq!(from as usize % Self::ERASE_SIZE, 0);
        self.0[from as usize..to as usize].fill(0xFF);
        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        assert_eq!(
            offset as usize % Self::WRITE_SIZE,
            0,
            "unaligned write at {offset:#x}"
        );
        assert_eq!(
            bytes.len() % Self::WRITE_SIZE,
            0,
            "partial write of {} bytes",
            bytes.len()
        );
        self.0[offset as usize..offset as usize + bytes.len()].copy_from_slice(bytes);
        Ok(())
    }
}

/// imgtool-like image: 0x800 header, payload, TLV area with a SHA-256 entry.
fn image(version: [u8; 3], payload_len: usize, hash_seed: u8) -> (Vec<u8>, [u8; 32]) {
    let hash: [u8; 32] = core::array::from_fn(|i| hash_seed.wrapping_add(i as u8));
    let mut img = vec![0u8; 0x800];
    img[0..4].copy_from_slice(&mcuboot::IMAGE_MAGIC.to_le_bytes());
    img[8..10].copy_from_slice(&0x800u16.to_le_bytes());
    img[12..16].copy_from_slice(&(payload_len as u32).to_le_bytes());
    img[20] = version[0];
    img[21] = version[1];
    img[22..24].copy_from_slice(&u16::from(version[2]).to_le_bytes());
    img.extend((0..payload_len).map(|i| (i * 7) as u8));
    img.extend_from_slice(&[0x07, 0x69, 40, 0, 0x10, 0x00, 32, 0]);
    img.extend_from_slice(&hash);
    (img, hash)
}

fn server_with_primary(version: [u8; 3]) -> (Server<FakeFlash>, [u8; 32]) {
    let mut flash = FakeFlash::new();
    let (img, hash) = image(version, 1000, 0x10);
    let start = MCU_BOOT_LAYOUT.primary_slot_start as usize;
    flash.0[start..start + img.len()].copy_from_slice(&img);
    (Server::new(flash, &MCU_BOOT_LAYOUT), hash)
}

fn request(
    op: u8,
    group: u16,
    id: u8,
    seq: u8,
    body: impl FnOnce(&mut Encoder<&mut Vec<u8>>),
) -> Vec<u8> {
    let mut cbor = Vec::new();
    body(&mut Encoder::new(&mut cbor));
    let mut frame = vec![0u8; HEADER_LEN];
    Header {
        op,
        version: 1,
        flags: 0,
        len: cbor.len() as u16,
        group,
        seq,
        id,
    }
    .write(&mut frame);
    frame.extend(cbor);
    frame
}

fn call(server: &mut Server<FakeFlash>, frame: &[u8]) -> (Header, Vec<u8>) {
    let mut out = [0u8; MAX_FRAME];
    let n = server.handle(frame, &mut out);
    let header = Header::parse(&out[..n]).unwrap();
    assert_eq!(usize::from(header.len), n - HEADER_LEN);
    (header, out[HEADER_LEN..n].to_vec())
}

/// Value of a top-level key, re-encoded so callers can decode it independently.
fn field<'a>(body: &'a [u8], key: &str) -> Option<Decoder<'a>> {
    let mut d = Decoder::new(body);
    let n = d.map().unwrap().unwrap();
    for _ in 0..n {
        let k = d.str().unwrap();
        if k == key {
            return Some(d);
        }
        d.skip().unwrap();
    }
    None
}

fn upload(server: &mut Server<FakeFlash>, img: &[u8], chunk: usize) -> u32 {
    let mut off = 0usize;
    let mut seq = 0u8;
    while off < img.len() {
        let end = (off + chunk).min(img.len());
        let frame = request(OP_WRITE, GROUP_IMAGE, IMAGE_UPLOAD, seq, |e| {
            let n = if off == 0 { 3 } else { 2 };
            e.map(n).unwrap();
            if off == 0 {
                e.str("len").unwrap().u32(img.len() as u32).unwrap();
            }
            e.str("off").unwrap().u32(off as u32).unwrap();
            e.str("data").unwrap().bytes(&img[off..end]).unwrap();
        });
        let (header, body) = call(server, &frame);
        assert_eq!(header.op, OP_WRITE + 1);
        assert_eq!(field(&body, "rc").unwrap().u8().unwrap(), rc::OK);
        off = field(&body, "off").unwrap().u32().unwrap() as usize;
        seq = seq.wrapping_add(1);
    }
    off as u32
}

struct Slot {
    slot: u8,
    hash: Vec<u8>,
    version: std::string::String,
    pending: bool,
    confirmed: bool,
    active: bool,
}

fn state(server: &mut Server<FakeFlash>) -> Vec<Slot> {
    let (_, body) = call(
        server,
        &request(OP_READ, GROUP_IMAGE, IMAGE_STATE, 0, |e| {
            e.map(0).unwrap();
        }),
    );
    let mut d = field(&body, "images").unwrap();
    let n = d.array().unwrap().unwrap();
    (0..n)
        .map(|_| {
            let mut slot = Slot {
                slot: 0,
                hash: Vec::new(),
                version: Default::default(),
                pending: false,
                confirmed: false,
                active: false,
            };
            let entries = d.map().unwrap().unwrap();
            for _ in 0..entries {
                match d.str().unwrap() {
                    "slot" => slot.slot = d.u8().unwrap(),
                    "hash" => slot.hash = d.bytes().unwrap().to_vec(),
                    "version" => slot.version = d.str().unwrap().into(),
                    "pending" => slot.pending = d.bool().unwrap(),
                    "confirmed" => slot.confirmed = d.bool().unwrap(),
                    "active" => slot.active = d.bool().unwrap(),
                    _ => d.skip().unwrap(),
                }
            }
            slot
        })
        .collect()
}

#[test]
fn header_roundtrip() {
    let h = Header {
        op: OP_WRITE,
        version: 1,
        flags: 0,
        len: 0x1234,
        group: GROUP_IMAGE,
        seq: 7,
        id: IMAGE_UPLOAD,
    };
    let mut b = [0u8; HEADER_LEN];
    h.write(&mut b);
    assert_eq!(b, [0x0A, 0, 0x12, 0x34, 0, 1, 7, 1]);
    assert_eq!(Header::parse(&b), Some(h));
}

#[test]
fn reassembles_fragmented_frames() {
    let frame = request(OP_READ, GROUP_OS, OS_ECHO, 3, |e| {
        e.map(1)
            .unwrap()
            .str("d")
            .unwrap()
            .str("hello world, fragmented")
            .unwrap();
    });
    let mut r = Reassembler::default();
    assert!(r.push(&frame[..5]).is_none());
    assert!(r.push(&frame[5..12]).is_none());
    assert_eq!(r.push(&frame[12..]).unwrap(), &frame[..]);
}

#[test]
fn echo_and_mcumgr_params() {
    let (mut server, _) = server_with_primary([1, 0, 0]);
    let (_, body) = call(
        &mut server,
        &request(OP_WRITE, GROUP_OS, OS_ECHO, 0, |e| {
            e.map(1).unwrap().str("d").unwrap().str("ping").unwrap();
        }),
    );
    assert_eq!(field(&body, "r").unwrap().str().unwrap(), "ping");

    let (_, body) = call(
        &mut server,
        &request(OP_READ, GROUP_OS, OS_MCUMGR_PARAMS, 1, |e| {
            e.map(0).unwrap();
        }),
    );
    assert_eq!(
        field(&body, "buf_size").unwrap().u32().unwrap(),
        MAX_FRAME as u32
    );
}

#[test]
fn fresh_primary_reports_confirmed_and_active() {
    let (mut server, hash) = server_with_primary([1, 2, 3]);
    let slots = state(&mut server);
    assert_eq!(slots.len(), 1);
    assert_eq!(slots[0].slot, 0);
    assert_eq!(slots[0].version, "1.2.3");
    assert_eq!(slots[0].hash, hash);
    assert!(slots[0].active && slots[0].confirmed && !slots[0].pending);
}

#[test]
fn upload_with_unaligned_chunks_lands_in_secondary_slot() {
    let (mut server, _) = server_with_primary([1, 0, 0]);
    let (img, hash) = image([1, 1, 0], 5000, 0x80);
    assert_eq!(upload(&mut server, &img, 333), img.len() as u32);

    let start = MCU_BOOT_LAYOUT.secondary_slot_start as usize;
    assert_eq!(&server.flash().0[start..start + img.len()], &img[..]);
    let slots = state(&mut server);
    assert_eq!(slots.len(), 2);
    assert_eq!(
        (
            slots[1].slot,
            slots[1].hash.as_slice(),
            slots[1].version.as_str()
        ),
        (1, &hash[..], "1.1.0")
    );
    assert!(!slots[1].pending);
}

#[test]
fn out_of_order_chunk_returns_expected_offset() {
    let (mut server, _) = server_with_primary([1, 0, 0]);
    let (img, _) = image([1, 1, 0], 2000, 0x80);
    let first = request(OP_WRITE, GROUP_IMAGE, IMAGE_UPLOAD, 0, |e| {
        e.map(3).unwrap();
        e.str("len").unwrap().u32(img.len() as u32).unwrap();
        e.str("off").unwrap().u32(0).unwrap();
        e.str("data").unwrap().bytes(&img[..256]).unwrap();
    });
    call(&mut server, &first);
    let skipped = request(OP_WRITE, GROUP_IMAGE, IMAGE_UPLOAD, 1, |e| {
        e.map(2).unwrap();
        e.str("off").unwrap().u32(1024).unwrap();
        e.str("data").unwrap().bytes(&img[1024..1280]).unwrap();
    });
    let (_, body) = call(&mut server, &skipped);
    assert_eq!(field(&body, "off").unwrap().u32().unwrap(), 256);
}

#[test]
fn upload_rejects_data_without_image_header() {
    let (mut server, _) = server_with_primary([1, 0, 0]);
    let frame = request(OP_WRITE, GROUP_IMAGE, IMAGE_UPLOAD, 0, |e| {
        e.map(3).unwrap();
        e.str("len").unwrap().u32(64).unwrap();
        e.str("off").unwrap().u32(0).unwrap();
        e.str("data").unwrap().bytes(&[0u8; 64]).unwrap();
    });
    let (_, body) = call(&mut server, &frame);
    let mut err = field(&body, "err").unwrap();
    assert_eq!(err.map().unwrap(), Some(2));
    assert_eq!(err.str().unwrap(), "group");
    assert_eq!(err.u16().unwrap(), GROUP_IMAGE);
    assert_eq!(err.str().unwrap(), "rc");
    assert_eq!(err.u8().unwrap(), rc::ECORRUPT);
}

#[test]
fn test_request_marks_secondary_pending_like_mcuboot() {
    let (mut server, _) = server_with_primary([1, 0, 0]);
    let (img, hash) = image([1, 1, 0], 3000, 0x80);
    upload(&mut server, &img, 400);

    let (_, body) = call(
        &mut server,
        &request(OP_WRITE, GROUP_IMAGE, IMAGE_STATE, 9, |e| {
            e.map(2).unwrap();
            e.str("hash").unwrap().bytes(&hash).unwrap();
            e.str("confirm").unwrap().bool(false).unwrap();
        }),
    );
    assert_eq!(
        field(&body, "images").unwrap().datatype().unwrap(),
        Type::Array
    );

    let base = MCU_BOOT_LAYOUT.secondary_slot_start;
    let size = MCU_BOOT_LAYOUT.secondary_slot_size;
    let flash = &server.flash().0;
    let magic = (base + mcuboot::magic_offset(size)) as usize;
    assert_eq!(flash[magic..magic + 16], mcuboot::BOOT_MAGIC);
    assert_eq!(
        flash[(base + mcuboot::swap_info_offset(size)) as usize],
        mcuboot::SWAP_TYPE_TEST
    );
    assert_eq!(
        flash[(base + mcuboot::image_ok_offset(size)) as usize],
        mcuboot::ERASED
    );
    assert!(state(&mut server)[1].pending);
}

#[test]
fn test_request_for_older_image_is_rejected() {
    let (mut server, _) = server_with_primary([1, 2, 0]);
    let (img, hash) = image([1, 1, 9], 3000, 0x80);
    upload(&mut server, &img, 400);

    let (_, body) = call(
        &mut server,
        &request(OP_WRITE, GROUP_IMAGE, IMAGE_STATE, 9, |e| {
            e.map(1).unwrap();
            e.str("hash").unwrap().bytes(&hash).unwrap();
        }),
    );
    let mut err = field(&body, "err").unwrap();
    assert_eq!(err.map().unwrap(), Some(2));
    assert_eq!(err.str().unwrap(), "group");
    assert_eq!(err.u16().unwrap(), GROUP_IMAGE);
    assert_eq!(err.str().unwrap(), "rc");
    assert_eq!(err.u8().unwrap(), rc::EBADSTATE);
    assert!(!state(&mut server)[1].pending);
}

#[test]
fn test_request_with_unknown_hash_is_rejected() {
    let (mut server, _) = server_with_primary([1, 0, 0]);
    let (img, _) = image([1, 1, 0], 3000, 0x80);
    upload(&mut server, &img, 400);
    let (_, body) = call(
        &mut server,
        &request(OP_WRITE, GROUP_IMAGE, IMAGE_STATE, 0, |e| {
            e.map(1)
                .unwrap()
                .str("hash")
                .unwrap()
                .bytes(&[0xAB; 32])
                .unwrap();
        }),
    );
    assert!(field(&body, "err").is_some());
}

#[test]
fn confirm_after_test_swap_sets_image_ok_in_primary() {
    let (mut server, _) = server_with_primary([1, 1, 0]);
    // State MCUboot leaves after a test swap: primary trailer has magic, image_ok unset.
    let base = MCU_BOOT_LAYOUT.primary_slot_start;
    let size = MCU_BOOT_LAYOUT.primary_slot_size;
    server
        .flash()
        .write(base + mcuboot::magic_offset(size), &mcuboot::BOOT_MAGIC)
        .unwrap();
    assert!(!state(&mut server)[0].confirmed);

    call(
        &mut server,
        &request(OP_WRITE, GROUP_IMAGE, IMAGE_STATE, 0, |e| {
            e.map(1)
                .unwrap()
                .str("confirm")
                .unwrap()
                .bool(true)
                .unwrap();
        }),
    );
    assert_eq!(
        server.flash().0[(base + mcuboot::image_ok_offset(size)) as usize],
        mcuboot::FLAG_SET
    );
    assert!(state(&mut server)[0].confirmed);
}

#[test]
fn self_confirm_marks_trial_image_permanent() {
    let (mut server, _) = server_with_primary([1, 1, 0]);
    assert!(server.running_image_confirmed());
    let base = MCU_BOOT_LAYOUT.primary_slot_start;
    let size = MCU_BOOT_LAYOUT.primary_slot_size;
    server
        .flash()
        .write(base + mcuboot::magic_offset(size), &mcuboot::BOOT_MAGIC)
        .unwrap();
    assert!(!server.running_image_confirmed());

    assert!(server.confirm_running_image());
    assert!(server.running_image_confirmed());
    assert!(server.confirm_running_image());
}

#[test]
fn reset_is_flagged_after_response() {
    let (mut server, _) = server_with_primary([1, 0, 0]);
    assert!(!server.reset_requested());
    let (header, _) = call(
        &mut server,
        &request(OP_WRITE, GROUP_OS, OS_RESET, 0, |e| {
            e.map(0).unwrap();
        }),
    );
    assert_eq!(header.op, OP_WRITE + 1);
    assert!(server.reset_requested());
}

#[test]
fn unknown_command_is_not_supported() {
    let (mut server, _) = server_with_primary([1, 0, 0]);
    let (_, body) = call(
        &mut server,
        &request(OP_READ, 63, 0, 0, |e| {
            e.map(0).unwrap();
        }),
    );
    let mut err = field(&body, "err").unwrap();
    err.map().unwrap();
    err.str().unwrap();
    err.u16().unwrap();
    err.str().unwrap();
    assert_eq!(err.u8().unwrap(), rc::ENOTSUP);
}
