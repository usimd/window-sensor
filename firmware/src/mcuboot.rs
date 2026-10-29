//! MCUboot image header / TLV parsing and swap-trailer layout as used by the SMP image group.
//!
//! Trailer layout (swap using scratch, image 0), measured back from the end of a slot:
//! `magic` (16 B), then `image_ok`, `copy_done`, `swap_info`, each one `MAX_ALIGN` block.

pub const IMAGE_MAGIC: u32 = 0x96f3_b83d;
pub const HEADER_LEN: usize = 32;
const TLV_INFO_MAGIC: u16 = 0x6907;
const TLV_PROT_INFO_MAGIC: u16 = 0x6908;
const TLV_SHA256: u16 = 0x10;
pub const SHA256_LEN: usize = 32;

/// MCUboot BOOT_MAX_ALIGN: defaults to the flash write-block-size, 16 for nRF54L RRAM in Zephyr DTS.
pub const MAX_ALIGN: u32 = 16;
pub const BOOT_MAGIC: [u8; 16] = [
    0x77, 0xc2, 0x95, 0xf3, 0x60, 0xd2, 0xef, 0x7f, 0x35, 0x52, 0x50, 0x0f, 0x2c, 0xb6, 0x79, 0x80,
];
pub const FLAG_SET: u8 = 0x01;
pub const ERASED: u8 = 0xFF;
pub const SWAP_TYPE_TEST: u8 = 2;
pub const SWAP_TYPE_PERM: u8 = 3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ImageVersion {
    pub major: u8,
    pub minor: u8,
    pub revision: u16,
    pub build: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ImageHeader {
    pub hdr_size: u16,
    pub protect_tlv_size: u16,
    pub img_size: u32,
    pub version: ImageVersion,
}

impl ImageHeader {
    pub fn parse(bytes: &[u8]) -> Option<Self> {
        let u16_at = |o: usize| u16::from_le_bytes([bytes[o], bytes[o + 1]]);
        let u32_at =
            |o: usize| u32::from_le_bytes([bytes[o], bytes[o + 1], bytes[o + 2], bytes[o + 3]]);
        if bytes.len() < HEADER_LEN || u32_at(0) != IMAGE_MAGIC {
            return None;
        }
        Some(Self {
            hdr_size: u16_at(8),
            protect_tlv_size: u16_at(10),
            img_size: u32_at(12),
            version: ImageVersion {
                major: bytes[20],
                minor: bytes[21],
                revision: u16_at(22),
                build: u32_at(24),
            },
        })
    }

    /// Offset of the unprotected TLV info header within the slot.
    pub const fn tlv_offset(&self) -> u32 {
        self.hdr_size as u32 + self.img_size + self.protect_tlv_size as u32
    }
}

/// Finds the SHA-256 TLV (the image hash SMP clients use to address images).
/// `read(offset, buf)` reads from the start of the slot.
pub fn find_sha256<E>(
    header: &ImageHeader,
    mut read: impl FnMut(u32, &mut [u8]) -> Result<(), E>,
) -> Result<Option<[u8; SHA256_LEN]>, E> {
    let start = header.tlv_offset();
    let mut info = [0u8; 4];
    read(start, &mut info)?;
    let magic = u16::from_le_bytes([info[0], info[1]]);
    if magic != TLV_INFO_MAGIC && magic != TLV_PROT_INFO_MAGIC {
        return Ok(None);
    }
    let end = start + u32::from(u16::from_le_bytes([info[2], info[3]]));
    let mut off = start + 4;
    while off + 4 <= end {
        let mut entry = [0u8; 4];
        read(off, &mut entry)?;
        let kind = u16::from_le_bytes([entry[0], entry[1]]);
        let len = u16::from_le_bytes([entry[2], entry[3]]);
        if kind == TLV_SHA256 && usize::from(len) == SHA256_LEN {
            let mut hash = [0u8; SHA256_LEN];
            read(off + 4, &mut hash)?;
            return Ok(Some(hash));
        }
        off += 4 + u32::from(len);
    }
    Ok(None)
}

pub const fn magic_offset(slot_size: u32) -> u32 {
    slot_size - BOOT_MAGIC.len() as u32
}

pub const fn image_ok_offset(slot_size: u32) -> u32 {
    magic_offset(slot_size) - MAX_ALIGN
}

pub const fn copy_done_offset(slot_size: u32) -> u32 {
    image_ok_offset(slot_size) - MAX_ALIGN
}

pub const fn swap_info_offset(slot_size: u32) -> u32 {
    copy_done_offset(slot_size) - MAX_ALIGN
}

/// One trailer field as written by MCUboot: the value followed by erased padding.
pub const fn flag_block(value: u8) -> [u8; MAX_ALIGN as usize] {
    let mut block = [ERASED; MAX_ALIGN as usize];
    block[0] = value;
    block
}

/// Swap info for image 0.
pub const fn swap_info(swap_type: u8) -> u8 {
    swap_type & 0x0F
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TrailerState {
    pub magic_good: bool,
    pub image_ok: bool,
    pub copy_done: bool,
    pub swap_type: u8,
}

impl TrailerState {
    pub fn read<E>(
        slot_size: u32,
        mut read: impl FnMut(u32, &mut [u8]) -> Result<(), E>,
    ) -> Result<Self, E> {
        let mut magic = [0u8; 16];
        read(magic_offset(slot_size), &mut magic)?;
        let mut byte = [0u8; 1];
        let mut field = |off: u32| -> Result<u8, E> {
            read(off, &mut byte)?;
            Ok(byte[0])
        };
        Ok(Self {
            magic_good: magic == BOOT_MAGIC,
            image_ok: field(image_ok_offset(slot_size))? == FLAG_SET,
            copy_done: field(copy_done_offset(slot_size))? == FLAG_SET,
            swap_type: field(swap_info_offset(slot_size))? & 0x0F,
        })
    }

    /// Primary slot: a fresh factory flash has no trailer and counts as confirmed.
    pub const fn primary_confirmed(&self) -> bool {
        !self.magic_good || self.image_ok
    }

    /// Secondary slot: marked for a test or permanent swap at the next boot.
    pub const fn secondary_pending(&self) -> bool {
        self.magic_good && (self.swap_type == SWAP_TYPE_TEST || self.swap_type == SWAP_TYPE_PERM)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SLOT: u32 = 0x60000;

    fn header_bytes(hdr_size: u16, img_size: u32, protect: u16) -> [u8; HEADER_LEN] {
        let mut h = [0u8; HEADER_LEN];
        h[0..4].copy_from_slice(&IMAGE_MAGIC.to_le_bytes());
        h[8..10].copy_from_slice(&hdr_size.to_le_bytes());
        h[10..12].copy_from_slice(&protect.to_le_bytes());
        h[12..16].copy_from_slice(&img_size.to_le_bytes());
        h[20] = 1;
        h[21] = 2;
        h[22..24].copy_from_slice(&3u16.to_le_bytes());
        h
    }

    #[test]
    fn parses_imgtool_header() {
        let header = ImageHeader::parse(&header_bytes(0x800, 0x1234, 0)).unwrap();
        assert_eq!(header.hdr_size, 0x800);
        assert_eq!(header.img_size, 0x1234);
        assert_eq!(
            (
                header.version.major,
                header.version.minor,
                header.version.revision
            ),
            (1, 2, 3)
        );
        assert_eq!(header.tlv_offset(), 0x800 + 0x1234);
    }

    #[test]
    fn rejects_missing_magic() {
        assert_eq!(ImageHeader::parse(&[0xFF; HEADER_LEN]), None);
    }

    #[test]
    fn finds_sha256_after_other_tlvs() {
        let header = ImageHeader::parse(&header_bytes(0x20, 0x10, 0)).unwrap();
        let mut slot = [0xFFu8; 0x100];
        let t = header.tlv_offset() as usize;
        let hash: [u8; 32] = core::array::from_fn(|i| i as u8);
        let tlvs: &[u8] = &[0x01, 0x00, 0x02, 0x00, 0xAA, 0xBB, 0x10, 0x00, 0x20, 0x00];
        let total = (4 + tlvs.len() + 32) as u16;
        slot[t..t + 4].copy_from_slice(&[0x07, 0x69, total as u8, (total >> 8) as u8]);
        slot[t + 4..t + 4 + tlvs.len()].copy_from_slice(tlvs);
        slot[t + 4 + tlvs.len()..t + 4 + tlvs.len() + 32].copy_from_slice(&hash);

        let read = |off: u32, buf: &mut [u8]| -> Result<(), ()> {
            buf.copy_from_slice(&slot[off as usize..off as usize + buf.len()]);
            Ok(())
        };
        assert_eq!(find_sha256(&header, read), Ok(Some(hash)));
    }

    #[test]
    fn trailer_offsets_step_back_by_max_align() {
        assert_eq!(magic_offset(SLOT), SLOT - 16);
        assert_eq!(image_ok_offset(SLOT), SLOT - 32);
        assert_eq!(copy_done_offset(SLOT), SLOT - 48);
        assert_eq!(swap_info_offset(SLOT), SLOT - 64);
    }

    #[test]
    fn erased_primary_counts_as_confirmed() {
        let state = TrailerState::read(SLOT, |_, buf: &mut [u8]| -> Result<(), ()> {
            buf.fill(ERASED);
            Ok(())
        })
        .unwrap();
        assert!(state.primary_confirmed());
        assert!(!state.secondary_pending());
    }

    #[test]
    fn secondary_with_magic_and_test_swap_is_pending() {
        let state = TrailerState::read(SLOT, |off, buf: &mut [u8]| -> Result<(), ()> {
            if off == magic_offset(SLOT) {
                buf.copy_from_slice(&BOOT_MAGIC);
            } else if off == swap_info_offset(SLOT) {
                buf[0] = swap_info(SWAP_TYPE_TEST);
            } else {
                buf.fill(ERASED);
            }
            Ok(())
        })
        .unwrap();
        assert!(state.secondary_pending());
        assert!(!state.primary_confirmed());
    }
}
