use crate::partition::SETTINGS_REGION;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};

pub const SOC_SETTINGS_MAGIC: u32 = 0x534f_4331;
pub const SOC_SETTINGS_VERSION: u16 = 1;
pub const SOC_SETTINGS_LEN: usize = 20;
pub const SOC_SETTINGS_OFFSET: u32 = SETTINGS_REGION.start;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), derive(defmt::Format))]
pub struct SocSettings {
    pub autocal_complete: bool,
    pub last_battery_percent: u8,
    pub best_resistance_mohm: u32,
    pub window_calibrated: bool,
    pub window_closed_baseline_mt_x10: u16,
}

impl Default for SocSettings {
    fn default() -> Self {
        Self {
            autocal_complete: false,
            last_battery_percent: 100,
            best_resistance_mohm: 0,
            window_calibrated: false,
            window_closed_baseline_mt_x10: 0,
        }
    }
}

impl SocSettings {
    pub fn window_closed_baseline_mt(self) -> Option<f32> {
        self.window_calibrated
            .then_some(self.window_closed_baseline_mt_x10 as f32 / 10.0)
    }
}

impl SocSettings {
    pub fn encode(self) -> [u8; SOC_SETTINGS_LEN] {
        let mut bytes = [0xff; SOC_SETTINGS_LEN];
        let magic = SOC_SETTINGS_MAGIC.to_le_bytes();
        let version = SOC_SETTINGS_VERSION.to_le_bytes();
        let flags = (self.autocal_complete as u8) | ((self.window_calibrated as u8) << 1);
        let resistance = self.best_resistance_mohm.to_le_bytes();
        let closed_baseline = self.window_closed_baseline_mt_x10.to_le_bytes();
        let checksum = checksum(self).to_le_bytes();

        bytes[0] = magic[0];
        bytes[1] = magic[1];
        bytes[2] = magic[2];
        bytes[3] = magic[3];
        bytes[4] = version[0];
        bytes[5] = version[1];
        bytes[6] = flags;
        bytes[7] = self.last_battery_percent;
        bytes[8] = resistance[0];
        bytes[9] = resistance[1];
        bytes[10] = resistance[2];
        bytes[11] = resistance[3];
        bytes[12] = closed_baseline[0];
        bytes[13] = closed_baseline[1];
        bytes[16] = checksum[0];
        bytes[17] = checksum[1];
        bytes[18] = checksum[2];
        bytes[19] = checksum[3];
        bytes
    }

    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < SOC_SETTINGS_LEN {
            return None;
        }

        let magic = u32::from_le_bytes(bytes[0..4].try_into().ok()?);
        let version = u16::from_le_bytes(bytes[4..6].try_into().ok()?);
        if magic != SOC_SETTINGS_MAGIC || version != SOC_SETTINGS_VERSION {
            return None;
        }

        let settings = Self {
            autocal_complete: (bytes[6] & 0x01) != 0,
            window_calibrated: (bytes[6] & 0x02) != 0,
            last_battery_percent: bytes[7],
            best_resistance_mohm: u32::from_le_bytes(bytes[8..12].try_into().ok()?),
            window_closed_baseline_mt_x10: u16::from_le_bytes(bytes[12..14].try_into().ok()?),
        };

        let stored_checksum = u32::from_le_bytes(bytes[16..20].try_into().ok()?);
        (stored_checksum == checksum(settings)).then_some(settings)
    }
}

pub fn checksum(settings: SocSettings) -> u32 {
    u32::from(settings.autocal_complete as u8)
        ^ u32::from(settings.window_calibrated as u8).rotate_left(3)
        ^ u32::from(settings.last_battery_percent)
        ^ settings.best_resistance_mohm.rotate_left(7)
        ^ u32::from(settings.window_closed_baseline_mt_x10).rotate_left(11)
        ^ SOC_SETTINGS_MAGIC
        ^ u32::from(SOC_SETTINGS_VERSION)
}

pub fn load_soc_settings<F: ReadNorFlash>(flash: &mut F) -> Option<SocSettings> {
    let mut bytes = [0xff; SOC_SETTINGS_LEN];
    flash.read(SOC_SETTINGS_OFFSET, &mut bytes).ok()?;
    SocSettings::decode(&bytes)
}

pub fn save_soc_settings<F>(flash: &mut F, settings: SocSettings) -> Result<(), F::Error>
where
    F: ReadNorFlash + NorFlash,
{
    if load_soc_settings(flash) == Some(settings) {
        return Ok(());
    }

    let page_start = SOC_SETTINGS_OFFSET;
    let page_end = page_start + <F as NorFlash>::ERASE_SIZE as u32;
    flash.erase(page_start, page_end)?;
    flash.write(SOC_SETTINGS_OFFSET, &settings.encode())
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_storage::nor_flash::{
        ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
    };

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum MockError {
        OutOfBounds,
        Unaligned,
    }

    impl NorFlashError for MockError {
        fn kind(&self) -> NorFlashErrorKind {
            match self {
                Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
                Self::Unaligned => NorFlashErrorKind::NotAligned,
            }
        }
    }

    struct MockFlash {
        bytes: [u8; 4096],
    }

    impl MockFlash {
        fn new() -> Self {
            Self {
                bytes: [0xff; 4096],
            }
        }
    }

    impl ErrorType for MockFlash {
        type Error = MockError;
    }

    impl ReadNorFlash for MockFlash {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let offset = offset as usize - SOC_SETTINGS_OFFSET as usize;
            let end = offset + bytes.len();
            if end > self.bytes.len() {
                return Err(MockError::OutOfBounds);
            }

            bytes.copy_from_slice(&self.bytes[offset..end]);
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.bytes.len()
        }
    }

    impl NorFlash for MockFlash {
        const WRITE_SIZE: usize = SOC_SETTINGS_LEN;
        const ERASE_SIZE: usize = 4096;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            if from != SOC_SETTINGS_OFFSET || to != SOC_SETTINGS_OFFSET + Self::ERASE_SIZE as u32 {
                return Err(MockError::Unaligned);
            }

            self.bytes.fill(0xff);
            Ok(())
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            if offset != SOC_SETTINGS_OFFSET || bytes.len() != Self::WRITE_SIZE {
                return Err(MockError::Unaligned);
            }

            self.bytes[..bytes.len()].copy_from_slice(bytes);
            Ok(())
        }
    }

    #[test]
    fn soc_settings_round_trip() {
        let settings = SocSettings {
            autocal_complete: true,
            last_battery_percent: 91,
            best_resistance_mohm: 12_345,
            window_calibrated: true,
            window_closed_baseline_mt_x10: 512,
        };

        assert_eq!(SocSettings::decode(&settings.encode()), Some(settings));
    }

    #[test]
    fn soc_settings_rejects_bad_magic() {
        let mut bytes = SocSettings::default().encode();
        bytes[0] = 0;

        assert_eq!(SocSettings::decode(&bytes), None);
    }

    #[test]
    fn soc_settings_rejects_bad_checksum() {
        let mut bytes = SocSettings {
            autocal_complete: true,
            last_battery_percent: 80,
            best_resistance_mohm: 9_000,
            window_calibrated: true,
            window_closed_baseline_mt_x10: 480,
        }
        .encode();
        bytes[16] ^= 0x01;

        assert_eq!(SocSettings::decode(&bytes), None);
    }

    #[test]
    fn save_and_load_soc_settings_round_trip() {
        let mut flash = MockFlash::new();
        let settings = SocSettings {
            autocal_complete: true,
            last_battery_percent: 87,
            best_resistance_mohm: 8_765,
            window_calibrated: true,
            window_closed_baseline_mt_x10: 505,
        };

        save_soc_settings(&mut flash, settings).unwrap();
        assert_eq!(load_soc_settings(&mut flash), Some(settings));
    }
}
