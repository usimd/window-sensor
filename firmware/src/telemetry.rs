//! Host-testable telemetry and BLE advertisement assembly.

use crate::bthome;
use crate::classifier::WindowState;

/// BLE AD type for Service Data - 16-bit UUID.
pub const AD_TYPE_SERVICE_DATA_UUID16: u8 = 0x16;

/// Shared environment reading model used by tasks and BLE encoding.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), derive(defmt::Format))]
pub struct EnvReading {
    /// SHT4x temperature in centi-degrees (2350 = 23.50 C)
    pub temperature_cdeg: i16,
    /// SHT4x relative humidity in centi-percent (6520 = 65.20 %RH)
    pub humidity_cpct: u16,
    /// Estimated battery SoC in percent (0-100)
    pub battery_soc_pct: u8,
}

/// Current user-visible window status.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), derive(defmt::Format))]
pub struct WindowStatus {
    pub state: WindowState,
    pub tampered: bool,
    pub problem: bool,
}

/// Monotonic packet counter used by BTHome for de-duplication.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PacketCounter {
    next: u8,
}

impl PacketCounter {
    pub const fn new() -> Self {
        Self { next: 0 }
    }

    pub fn next_packet_id(&mut self) -> u8 {
        let packet_id = self.next;
        self.next = self.next.wrapping_add(1);
        packet_id
    }
}

/// Encode a BTHome state advertisement as a complete BLE service-data AD structure.
pub fn encode_state_advertisement(buf: &mut [u8], packet_id: u8, status: WindowStatus) -> usize {
    encode_service_data_ad(buf, |payload| {
        bthome::encode_state_event(
            payload,
            packet_id,
            status.state != WindowState::Closed,
            status.state == WindowState::Tilt,
            status.tampered,
            status.problem,
        )
    })
}

/// Encode a periodic heartbeat advertisement as a complete BLE service-data AD structure.
pub fn encode_heartbeat_advertisement(
    buf: &mut [u8],
    packet_id: u8,
    env: EnvReading,
    status: WindowStatus,
) -> usize {
    encode_service_data_ad(buf, |payload| {
        bthome::encode_heartbeat(
            payload,
            packet_id,
            env.battery_soc_pct,
            env.temperature_cdeg,
            env.humidity_cpct,
            status.state != WindowState::Closed,
            status.state == WindowState::Tilt,
            status.tampered,
            status.problem,
        )
    })
}

/// Encode a BTHome button-event advertisement as a complete BLE service-data AD structure.
pub fn encode_button_advertisement(
    buf: &mut [u8],
    packet_id: u8,
    event: bthome::ButtonEvent,
) -> usize {
    encode_service_data_ad(buf, |payload| {
        bthome::encode_button_event(payload, packet_id, event)
    })
}

fn encode_service_data_ad(
    buf: &mut [u8],
    encode_payload: impl FnOnce(&mut [u8]) -> usize,
) -> usize {
    debug_assert!(buf.len() >= 4);

    let payload_len = encode_payload(&mut buf[4..]);
    buf[0] = (payload_len + 3) as u8;
    buf[1] = AD_TYPE_SERVICE_DATA_UUID16;
    buf[2] = bthome::BTHOME_UUID_LE[0];
    buf[3] = bthome::BTHOME_UUID_LE[1];
    payload_len + 4
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packet_counter_wraps() {
        let mut counter = PacketCounter { next: 254 };
        assert_eq!(counter.next_packet_id(), 254);
        assert_eq!(counter.next_packet_id(), 255);
        assert_eq!(counter.next_packet_id(), 0);
        assert_eq!(counter.next_packet_id(), 1);
    }

    #[test]
    fn state_advertisement_has_bthome_service_data_prefix() {
        let mut buf = [0u8; 32];
        let len = encode_state_advertisement(
            &mut buf,
            7,
            WindowStatus {
                state: WindowState::Tilt,
                tampered: true,
                problem: false,
            },
        );

        assert_eq!(len, 15);
        assert_eq!(buf[0], 14);
        assert_eq!(buf[1], AD_TYPE_SERVICE_DATA_UUID16);
        assert_eq!(&buf[2..4], &bthome::BTHOME_UUID_LE);
        assert_eq!(buf[4], bthome::DEVICE_INFO);
        assert_eq!(buf[5], bthome::obj_id::PACKET_ID);
        assert_eq!(buf[6], 7);
        assert_eq!(buf[7], bthome::obj_id::GENERIC_BOOLEAN);
        assert_eq!(buf[8], 1);
        assert_eq!(buf[9], bthome::obj_id::PROBLEM);
        assert_eq!(buf[10], 0);
        assert_eq!(buf[11], bthome::obj_id::TAMPER);
        assert_eq!(buf[12], 1);
        assert_eq!(buf[13], bthome::obj_id::WINDOW);
        assert_eq!(buf[14], 1);
    }

    #[test]
    fn heartbeat_advertisement_fits_single_ble_packet() {
        let mut buf = [0u8; 32];
        let len = encode_heartbeat_advertisement(
            &mut buf,
            42,
            EnvReading {
                temperature_cdeg: 2350,
                humidity_cpct: 6520,
                battery_soc_pct: 88,
            },
            WindowStatus {
                state: WindowState::Closed,
                tampered: false,
                problem: false,
            },
        );

        assert_eq!(buf[1], AD_TYPE_SERVICE_DATA_UUID16);
        assert_eq!(&buf[2..4], &bthome::BTHOME_UUID_LE);
        assert_eq!(buf[4], bthome::DEVICE_INFO);
        assert!(len <= 31, "advertisement length {} exceeds BLE limit", len);
    }

    #[test]
    fn button_advertisement_contains_button_event() {
        let mut buf = [0u8; 32];
        let len = encode_button_advertisement(&mut buf, 11, bthome::ButtonEvent::TriplePress);

        assert_eq!(len, 9);
        assert_eq!(buf[1], AD_TYPE_SERVICE_DATA_UUID16);
        assert_eq!(&buf[2..4], &bthome::BTHOME_UUID_LE);
        assert_eq!(buf[4], bthome::DEVICE_INFO);
        assert_eq!(buf[5], bthome::obj_id::PACKET_ID);
        assert_eq!(buf[6], 11);
        assert_eq!(buf[7], bthome::obj_id::BUTTON_EVENT);
        assert_eq!(buf[8], 0x03);
    }
}
