//! Update window: a short connectable advertising phase so OTA clients (Home Assistant) can connect.
//!
//! The normal adverts are non-connectable to save power. Once per period the device advertises
//! connectably with a scan response carrying its `WS-xxxx` name, which is what the HA integration
//! matches on. The window stays open while a client is connected, up to a hard limit.

pub const WINDOW_PERIOD_S: u32 = 3600;
pub const WINDOW_DURATION_MS: u32 = 5000;
pub const WINDOW_ADV_INTERVAL_MS: u32 = 100;
pub const MAX_CONNECTION_S: u32 = 300;

pub const AD_TYPE_FLAGS: u8 = 0x01;
pub const AD_TYPE_COMPLETE_LOCAL_NAME: u8 = 0x09;
/// LE General Discoverable, BR/EDR not supported.
pub const FLAGS_LE_GENERAL_DISCOVERABLE: u8 = 0x06;
pub const NAME_PREFIX: &[u8; 3] = b"WS-";
pub const NAME_LEN: usize = NAME_PREFIX.len() + 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WindowSchedule {
    next_at_ms: u64,
}

impl WindowSchedule {
    /// The first window opens one period after boot so a freshly flashed or rebooted device
    /// does not immediately accept connections.
    pub const fn new(now_ms: u64) -> Self {
        Self {
            next_at_ms: now_ms + WINDOW_PERIOD_S as u64 * 1000,
        }
    }

    pub const fn due(&self, now_ms: u64) -> bool {
        now_ms >= self.next_at_ms
    }

    /// Call when a window was opened (or skipped because prerequisites failed).
    pub fn advance(&mut self, now_ms: u64) {
        while self.next_at_ms <= now_ms {
            self.next_at_ms += WINDOW_PERIOD_S as u64 * 1000;
        }
    }
}

/// `WS-` plus the two least significant address bytes as upper-case hex (`C0:..:1A:2B` -> `WS-1A2B`).
pub fn local_name(addr_le: [u8; 6]) -> [u8; NAME_LEN] {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let mut name = [0u8; NAME_LEN];
    name[..3].copy_from_slice(NAME_PREFIX);
    for (i, byte) in [addr_le[1], addr_le[0]].into_iter().enumerate() {
        name[3 + 2 * i] = HEX[usize::from(byte >> 4)];
        name[4 + 2 * i] = HEX[usize::from(byte & 0x0F)];
    }
    name
}

/// Flags AD followed by an existing AD structure (e.g. the BTHome heartbeat); returns the length.
pub fn encode_connectable_advertisement(buf: &mut [u8], ad: &[u8]) -> usize {
    buf[..3].copy_from_slice(&[2, AD_TYPE_FLAGS, FLAGS_LE_GENERAL_DISCOVERABLE]);
    buf[3..3 + ad.len()].copy_from_slice(ad);
    3 + ad.len()
}

pub fn encode_scan_response(buf: &mut [u8], addr_le: [u8; 6]) -> usize {
    let name = local_name(addr_le);
    buf[0] = (NAME_LEN + 1) as u8;
    buf[1] = AD_TYPE_COMPLETE_LOCAL_NAME;
    buf[2..2 + NAME_LEN].copy_from_slice(&name);
    2 + NAME_LEN
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::classifier::WindowState;
    use crate::telemetry::{EnvReading, WindowStatus, encode_heartbeat_advertisement};

    #[test]
    fn first_window_opens_after_one_period() {
        let schedule = WindowSchedule::new(1_000);
        assert!(!schedule.due(1_000));
        assert!(schedule.due(1_000 + WINDOW_PERIOD_S as u64 * 1000));
    }

    #[test]
    fn advance_skips_missed_periods() {
        let period_ms = WINDOW_PERIOD_S as u64 * 1000;
        let mut schedule = WindowSchedule::new(0);
        schedule.advance(3 * period_ms + 5);
        assert!(!schedule.due(3 * period_ms + 6));
        assert!(schedule.due(4 * period_ms));
    }

    #[test]
    fn name_uses_two_least_significant_address_bytes() {
        let addr_le = [0x2B, 0x1A, 0x33, 0x44, 0x55, 0xC6];
        assert_eq!(&local_name(addr_le), b"WS-1A2B");
    }

    #[test]
    fn scan_response_is_complete_local_name_ad() {
        let mut buf = [0u8; 31];
        let len = encode_scan_response(&mut buf, [0x0F, 0xA0, 0, 0, 0, 0xC0]);
        assert_eq!(&buf[..len], b"\x08\x09WS-A00F");
    }

    #[test]
    fn connectable_heartbeat_fits_legacy_advert() {
        let mut heartbeat = [0u8; 31];
        let ad_len = encode_heartbeat_advertisement(
            &mut heartbeat,
            255,
            EnvReading {
                temperature_cdeg: -4000,
                humidity_cpct: 10000,
                battery_soc_pct: 100,
            },
            WindowStatus {
                state: WindowState::Tilt,
                tampered: true,
                problem: true,
            },
        );
        let mut adv = [0u8; 31];
        let len = encode_connectable_advertisement(&mut adv, &heartbeat[..ad_len]);
        assert!(len <= 31, "connectable advert is {len} bytes");
        assert_eq!(adv[..3], [2, AD_TYPE_FLAGS, FLAGS_LE_GENERAL_DISCOVERABLE]);
        assert_eq!(adv[3..len], heartbeat[..ad_len]);
    }
}
