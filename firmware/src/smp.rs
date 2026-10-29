//! SMP (MCUmgr) server core: framing plus the OS and image-management commands an OTA client
//! (smpclient / Home Assistant) needs. Transport-agnostic and host-tested; the BLE GATT service
//! feeds `Reassembler` with written fragments and notifies the response frames.
//!
//! Supported: OS echo (0/0), reset (0/5), MCUmgr parameters (0/6); image state read/write (1/0)
//! and upload (1/1) into the secondary slot of image 0.

use embedded_storage::nor_flash::NorFlash;
use minicbor::encode::write::Cursor;
use minicbor::{Decoder, Encoder};

use crate::mcuboot::{self, ImageHeader, SHA256_LEN, TrailerState};
use crate::partition::{PartitionLayout, PartitionRegion};

pub const HEADER_LEN: usize = 8;
/// Largest SMP frame accepted (header + CBOR); advertised via MCUmgr parameters.
pub const MAX_FRAME: usize = 512;

pub const OP_READ: u8 = 0;
pub const OP_WRITE: u8 = 2;
pub const GROUP_OS: u16 = 0;
pub const GROUP_IMAGE: u16 = 1;
pub const OS_ECHO: u8 = 0;
pub const OS_RESET: u8 = 5;
pub const OS_MCUMGR_PARAMS: u8 = 6;
pub const IMAGE_STATE: u8 = 0;
pub const IMAGE_UPLOAD: u8 = 1;

/// MCUmgr result codes.
pub mod rc {
    pub const OK: u8 = 0;
    pub const UNKNOWN: u8 = 1;
    pub const ENOMEM: u8 = 2;
    pub const EINVAL: u8 = 3;
    pub const ENOENT: u8 = 5;
    pub const EBADSTATE: u8 = 6;
    pub const ENOTSUP: u8 = 8;
    pub const ECORRUPT: u8 = 9;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    pub op: u8,
    pub version: u8,
    pub flags: u8,
    pub len: u16,
    pub group: u16,
    pub seq: u8,
    pub id: u8,
}

impl Header {
    pub fn parse(b: &[u8]) -> Option<Self> {
        if b.len() < HEADER_LEN {
            return None;
        }
        Some(Self {
            op: b[0] & 0x07,
            version: (b[0] >> 3) & 0x03,
            flags: b[1],
            len: u16::from_be_bytes([b[2], b[3]]),
            group: u16::from_be_bytes([b[4], b[5]]),
            seq: b[6],
            id: b[7],
        })
    }

    pub fn write(&self, b: &mut [u8]) {
        b[0] = (self.version << 3) | self.op;
        b[1] = self.flags;
        b[2..4].copy_from_slice(&self.len.to_be_bytes());
        b[4..6].copy_from_slice(&self.group.to_be_bytes());
        b[6] = self.seq;
        b[7] = self.id;
    }
}

/// Collects GATT write fragments into complete SMP frames.
pub struct Reassembler {
    buf: [u8; MAX_FRAME],
    len: usize,
}

impl Default for Reassembler {
    fn default() -> Self {
        Self {
            buf: [0; MAX_FRAME],
            len: 0,
        }
    }
}

impl Reassembler {
    /// Returns the complete frame once all of its bytes arrived. Oversized frames are dropped.
    pub fn push(&mut self, fragment: &[u8]) -> Option<&[u8]> {
        if self.len + fragment.len() > MAX_FRAME {
            self.len = 0;
            return None;
        }
        self.buf[self.len..self.len + fragment.len()].copy_from_slice(fragment);
        self.len += fragment.len();
        let header = Header::parse(&self.buf[..self.len])?;
        let total = HEADER_LEN + usize::from(header.len);
        if total > MAX_FRAME {
            self.len = 0;
            return None;
        }
        if self.len < total {
            return None;
        }
        self.len = 0;
        Some(&self.buf[..total])
    }
}

#[derive(Clone, Copy, Debug)]
struct Upload {
    len: u32,
    off: u32,
    staged: [u8; STAGE],
    staged_len: usize,
    erased_up_to: u32,
}

const STAGE: usize = 32;

type EncResult = Result<(), minicbor::encode::Error<minicbor::encode::write::EndOfSlice>>;

/// SMP request handler backed by the MCUboot slots in `flash` (absolute addresses).
pub struct Server<F> {
    flash: F,
    primary: PartitionRegion,
    secondary: PartitionRegion,
    upload: Option<Upload>,
    reset_requested: bool,
}

impl<F: NorFlash> Server<F> {
    pub fn new(flash: F, layout: &PartitionLayout) -> Self {
        assert!(F::WRITE_SIZE <= STAGE && STAGE.is_multiple_of(F::WRITE_SIZE));
        Self {
            flash,
            primary: layout.primary_slot_region(),
            secondary: layout.secondary_slot_region(),
            upload: None,
            reset_requested: false,
        }
    }

    /// Set after an OS reset request was answered; the transport resets once the response is out.
    pub fn reset_requested(&self) -> bool {
        self.reset_requested
    }

    pub fn flash(&mut self) -> &mut F {
        &mut self.flash
    }

    /// Handles one complete frame and writes the response frame into `out`; returns its length.
    pub fn handle(&mut self, frame: &[u8], out: &mut [u8]) -> usize {
        let Some(req) = Header::parse(frame) else {
            return 0;
        };
        let body = &frame[HEADER_LEN..];
        let mut cursor = Cursor::new(&mut out[HEADER_LEN..]);
        let mut enc = Encoder::new(&mut cursor);
        let result = match (req.group, req.id, req.op) {
            (GROUP_OS, OS_ECHO, _) => self.echo(body, &mut enc),
            (GROUP_OS, OS_RESET, OP_WRITE) => {
                self.reset_requested = true;
                enc.map(0).map(|_| ())
            }
            (GROUP_OS, OS_MCUMGR_PARAMS, OP_READ) => enc
                .map(2)
                .and_then(|e| e.str("buf_size")?.u32(MAX_FRAME as u32))
                .and_then(|e| e.str("buf_count")?.u32(1))
                .map(|_| ()),
            (GROUP_IMAGE, IMAGE_STATE, OP_READ) => self.image_state(&mut enc),
            (GROUP_IMAGE, IMAGE_STATE, OP_WRITE) => match self.image_state_write(body) {
                rc::OK => self.image_state(&mut enc),
                code => error(&mut enc, req, code),
            },
            (GROUP_IMAGE, IMAGE_UPLOAD, OP_WRITE) => match self.upload(body) {
                Ok(off) => enc
                    .map(2)
                    .and_then(|e| e.str("rc")?.u8(rc::OK)?.str("off")?.u32(off))
                    .map(|_| ()),
                Err(code) => error(&mut enc, req, code),
            },
            _ => error(&mut enc, req, rc::ENOTSUP),
        };
        let body_len = if result.is_ok() {
            cursor.position()
        } else {
            let mut cursor = Cursor::new(&mut out[HEADER_LEN..]);
            let _ = error(&mut Encoder::new(&mut cursor), req, rc::ENOMEM);
            cursor.position()
        };
        Header {
            op: req.op + 1,
            flags: 0,
            len: body_len as u16,
            ..req
        }
        .write(out);
        HEADER_LEN + body_len
    }

    fn echo<W: minicbor::encode::Write<Error = minicbor::encode::write::EndOfSlice>>(
        &mut self,
        body: &[u8],
        enc: &mut Encoder<W>,
    ) -> EncResult {
        let mut text = "";
        let _ = for_each_entry(body, |key, d| {
            if key == "d" {
                text = d.str()?;
            } else {
                d.skip()?;
            }
            Ok(())
        });
        enc.map(1)?.str("r")?.str(text)?;
        Ok(())
    }

    fn slot_info(&mut self, region: PartitionRegion) -> Option<(ImageHeader, [u8; SHA256_LEN])> {
        let mut raw = [0u8; mcuboot::HEADER_LEN];
        self.flash.read(region.start, &mut raw).ok()?;
        let header = ImageHeader::parse(&raw)?;
        if header.tlv_offset() >= region.size {
            return None;
        }
        let flash = &mut self.flash;
        let hash = mcuboot::find_sha256(&header, |off, buf| flash.read(region.start + off, buf))
            .ok()??;
        Some((header, hash))
    }

    fn trailer(&mut self, region: PartitionRegion) -> Option<TrailerState> {
        let flash = &mut self.flash;
        TrailerState::read(region.size, |off, buf| flash.read(region.start + off, buf)).ok()
    }

    fn image_state<W: minicbor::encode::Write<Error = minicbor::encode::write::EndOfSlice>>(
        &mut self,
        enc: &mut Encoder<W>,
    ) -> EncResult {
        let primary = self.slot_info(self.primary);
        let secondary = self
            .upload
            .is_none()
            .then(|| self.slot_info(self.secondary))
            .flatten();
        let primary_trailer = self.trailer(self.primary);
        let secondary_trailer = self.trailer(self.secondary);

        let count = u64::from(primary.is_some()) + u64::from(secondary.is_some());
        enc.map(2)?.str("images")?.array(count)?;
        if let Some((header, hash)) = primary {
            let confirmed = primary_trailer.is_none_or(|t| t.primary_confirmed());
            image_entry(enc, 0, &header, &hash, true, false, confirmed)?;
        }
        if let Some((header, hash)) = secondary {
            let pending = secondary_trailer.is_some_and(|t| t.secondary_pending());
            image_entry(enc, 1, &header, &hash, false, pending, false)?;
        }
        enc.str("splitStatus")?.u8(0)?;
        Ok(())
    }

    fn image_state_write(&mut self, body: &[u8]) -> u8 {
        let mut hash: Option<[u8; SHA256_LEN]> = None;
        let mut confirm = false;
        let parsed = for_each_entry(body, |key, d| {
            match key {
                "hash" => hash = d.bytes()?.try_into().ok(),
                "confirm" => confirm = d.bool()?,
                _ => d.skip()?,
            }
            Ok(())
        });
        if parsed.is_err() {
            return rc::EINVAL;
        }

        let primary_hash = self.slot_info(self.primary).map(|(_, h)| h);
        if confirm && (hash.is_none() || hash == primary_hash) {
            return self.confirm_primary();
        }
        let Some((_, secondary_hash)) = self.slot_info(self.secondary) else {
            return rc::ENOENT;
        };
        if hash != Some(secondary_hash) || self.upload.is_some() {
            return rc::EINVAL;
        }
        let swap_type = if confirm {
            mcuboot::SWAP_TYPE_PERM
        } else {
            mcuboot::SWAP_TYPE_TEST
        };
        self.mark_pending(swap_type)
    }

    fn confirm_primary(&mut self) -> u8 {
        let Some(trailer) = self.trailer(self.primary) else {
            return rc::UNKNOWN;
        };
        if trailer.primary_confirmed() {
            return rc::OK;
        }
        let off = self.primary.start + mcuboot::image_ok_offset(self.primary.size);
        match self
            .flash
            .write(off, &mcuboot::flag_block(mcuboot::FLAG_SET))
        {
            Ok(()) => rc::OK,
            Err(_) => rc::UNKNOWN,
        }
    }

    /// Mirrors MCUboot boot_set_next(): magic, optional image_ok, then swap_info.
    fn mark_pending(&mut self, swap_type: u8) -> u8 {
        let base = self.secondary.start;
        let size = self.secondary.size;
        let mut ok = self
            .flash
            .write(base + mcuboot::magic_offset(size), &mcuboot::BOOT_MAGIC)
            .is_ok();
        if swap_type == mcuboot::SWAP_TYPE_PERM {
            ok &= self
                .flash
                .write(
                    base + mcuboot::image_ok_offset(size),
                    &mcuboot::flag_block(mcuboot::FLAG_SET),
                )
                .is_ok();
        }
        ok &= self
            .flash
            .write(
                base + mcuboot::swap_info_offset(size),
                &mcuboot::flag_block(mcuboot::swap_info(swap_type)),
            )
            .is_ok();
        if ok { rc::OK } else { rc::UNKNOWN }
    }

    /// Returns the offset the client should continue at.
    fn upload(&mut self, body: &[u8]) -> Result<u32, u8> {
        let mut off: Option<u32> = None;
        let mut len: Option<u32> = None;
        let mut image = 0u32;
        let mut data: &[u8] = &[];
        for_each_entry(body, |key, d| {
            match key {
                "off" => off = Some(d.u32()?),
                "len" => len = Some(d.u32()?),
                "image" => image = d.u32()?,
                "data" => data = d.bytes()?,
                _ => d.skip()?,
            }
            Ok(())
        })
        .map_err(|_| rc::EINVAL)?;
        let off = off.ok_or(rc::EINVAL)?;
        if image != 0 {
            return Err(rc::ENOENT);
        }

        if off == 0 {
            self.start_upload(len.ok_or(rc::EINVAL)?, data)?;
        }
        let upload = self.upload.as_mut().ok_or(rc::EBADSTATE)?;
        if off != upload.off {
            return Ok(upload.off);
        }
        if off + data.len() as u32 > upload.len {
            return Err(rc::EINVAL);
        }
        self.append(data)?;
        let upload = self.upload.as_ref().ok_or(rc::EBADSTATE)?;
        let next = upload.off;
        if next == upload.len {
            self.finish_upload()?;
        }
        Ok(next)
    }

    fn start_upload(&mut self, len: u32, first: &[u8]) -> Result<(), u8> {
        let max = self.secondary.size - PartitionLayout::IMAGE_TRAILER_RESERVE;
        if len == 0 || len > max {
            return Err(rc::ENOMEM);
        }
        if ImageHeader::parse(first).is_none() {
            return Err(rc::ECORRUPT);
        }
        // Drop any stale swap request before writing a new image.
        let trailer_page = self.secondary.start + self.secondary.size - F::ERASE_SIZE as u32;
        self.flash
            .erase(trailer_page, trailer_page + F::ERASE_SIZE as u32)
            .map_err(|_| rc::UNKNOWN)?;
        self.upload = Some(Upload {
            len,
            off: 0,
            staged: [0xFF; STAGE],
            staged_len: 0,
            erased_up_to: self.secondary.start,
        });
        Ok(())
    }

    fn append(&mut self, mut data: &[u8]) -> Result<(), u8> {
        while !data.is_empty() {
            let upload = self.upload.as_mut().ok_or(rc::EBADSTATE)?;
            let take = data.len().min(STAGE - upload.staged_len);
            upload.staged[upload.staged_len..upload.staged_len + take]
                .copy_from_slice(&data[..take]);
            upload.staged_len += take;
            upload.off += take as u32;
            data = &data[take..];
            if upload.staged_len == STAGE {
                self.flush(STAGE)?;
            }
        }
        Ok(())
    }

    /// Writes `n` staged bytes (a multiple of the write size) at the stage's slot offset.
    fn flush(&mut self, n: usize) -> Result<(), u8> {
        let upload = self.upload.as_mut().ok_or(rc::EBADSTATE)?;
        let addr = self.secondary.start + upload.off - upload.staged_len as u32;
        let end = addr + n as u32;
        while upload.erased_up_to < end {
            let page = upload.erased_up_to;
            self.flash
                .erase(page, page + F::ERASE_SIZE as u32)
                .map_err(|_| rc::UNKNOWN)?;
            upload.erased_up_to += F::ERASE_SIZE as u32;
        }
        self.flash
            .write(addr, &upload.staged[..n])
            .map_err(|_| rc::UNKNOWN)?;
        upload.staged = [0xFF; STAGE];
        upload.staged_len = 0;
        Ok(())
    }

    fn finish_upload(&mut self) -> Result<(), u8> {
        let staged = self.upload.as_ref().ok_or(rc::EBADSTATE)?.staged_len;
        if staged > 0 {
            let padded = staged.div_ceil(F::WRITE_SIZE) * F::WRITE_SIZE;
            self.flush(padded)?;
        }
        self.upload = None;
        match self.slot_info(self.secondary) {
            Some(_) => Ok(()),
            None => Err(rc::ECORRUPT),
        }
    }
}

fn for_each_entry<'b>(
    body: &'b [u8],
    mut f: impl FnMut(&str, &mut Decoder<'b>) -> Result<(), minicbor::decode::Error>,
) -> Result<(), minicbor::decode::Error> {
    let mut d = Decoder::new(body);
    let n = d
        .map()?
        .ok_or_else(|| minicbor::decode::Error::message("indefinite map"))?;
    for _ in 0..n {
        let key = d.str()?;
        f(key, &mut d)?;
    }
    Ok(())
}

fn error<W: minicbor::encode::Write<Error = minicbor::encode::write::EndOfSlice>>(
    enc: &mut Encoder<W>,
    req: Header,
    code: u8,
) -> EncResult {
    if req.version >= 1 {
        enc.map(1)?
            .str("err")?
            .map(2)?
            .str("group")?
            .u16(req.group)?
            .str("rc")?
            .u8(code)?;
    } else {
        enc.map(1)?.str("rc")?.u8(code)?;
    }
    Ok(())
}

fn image_entry<W: minicbor::encode::Write<Error = minicbor::encode::write::EndOfSlice>>(
    enc: &mut Encoder<W>,
    slot: u8,
    header: &ImageHeader,
    hash: &[u8; SHA256_LEN],
    active: bool,
    pending: bool,
    confirmed: bool,
) -> EncResult {
    let mut version = VersionText::default();
    let v = header.version;
    let _ = core::fmt::write(
        &mut version,
        format_args!("{}.{}.{}", v.major, v.minor, v.revision),
    );
    enc.map(9)?
        .str("image")?
        .u8(0)?
        .str("slot")?
        .u8(slot)?
        .str("version")?
        .str(version.as_str())?
        .str("hash")?
        .bytes(hash)?
        .str("bootable")?
        .bool(true)?
        .str("pending")?
        .bool(pending)?
        .str("confirmed")?
        .bool(confirmed)?
        .str("active")?
        .bool(active)?
        .str("permanent")?
        .bool(false)?;
    Ok(())
}

#[derive(Default)]
struct VersionText {
    buf: [u8; 16],
    len: usize,
}

impl VersionText {
    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
}

impl core::fmt::Write for VersionText {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let end = self.len + s.len();
        if end > self.buf.len() {
            return Err(core::fmt::Error);
        }
        self.buf[self.len..end].copy_from_slice(s.as_bytes());
        self.len = end;
        Ok(())
    }
}

#[cfg(test)]
mod tests;
