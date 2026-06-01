//! BTHome v2 payload encoder — pure logic, no hardware dependencies.
//!
//! Encodes window state, environmental data, and device status into
//! BTHome v2 service data format for BLE advertising.

/// BTHome v2 object IDs
pub mod obj_id {
    pub const PACKET_ID: u8 = 0x00;
    pub const BATTERY: u8 = 0x01;
    pub const TEMPERATURE: u8 = 0x02; // sint16, factor 0.01 °C
    pub const HUMIDITY: u8 = 0x03; // uint16, factor 0.01 %
    pub const GENERIC_BOOLEAN: u8 = 0x0F;
    pub const PROBLEM: u8 = 0x26;
    pub const WINDOW: u8 = 0x2D; // 0=closed, 1=open
    pub const TAMPER: u8 = 0x2B; // 0=normal, 1=tampered
    pub const BUTTON_EVENT: u8 = 0x3A;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), derive(defmt::Format))]
pub enum ButtonEvent {
    Press,
    DoublePress,
    TriplePress,
    LongPress,
    HoldPress,
}

impl ButtonEvent {
    pub const fn event_id(self) -> u8 {
        match self {
            Self::Press => 0x01,
            Self::DoublePress => 0x02,
            Self::TriplePress => 0x03,
            Self::LongPress => 0x04,
            Self::HoldPress => 0x80,
        }
    }
}

/// BTHome v2 device information byte
/// Bits 7-5: version (010 = v2)
/// Bit 2: trigger-based (1 = yes)
/// Bit 0: encryption (0 = no)
pub const DEVICE_INFO: u8 = 0x44; // 0b0100_0100

/// BTHome UUID (little-endian for BLE advertisement)
pub const BTHOME_UUID_LE: [u8; 2] = [0xD2, 0xFC]; // 0xFCD2 in LE

/// Encode a window state-change event into BTHome v2 payload.
/// Returns the number of bytes written to `buf`.
pub fn encode_state_event(
    buf: &mut [u8],
    packet_id: u8,
    window_open: bool,
    is_tilt: bool,
    tampered: bool,
    problem: bool,
) -> usize {
    let mut pos = 0;

    buf[pos] = DEVICE_INFO;
    pos += 1;

    buf[pos] = obj_id::PACKET_ID;
    buf[pos + 1] = packet_id;
    pos += 2;

    buf[pos] = obj_id::GENERIC_BOOLEAN;
    buf[pos + 1] = is_tilt as u8;
    pos += 2;

    buf[pos] = obj_id::PROBLEM;
    buf[pos + 1] = problem as u8;
    pos += 2;

    buf[pos] = obj_id::TAMPER;
    buf[pos + 1] = tampered as u8;
    pos += 2;

    buf[pos] = obj_id::WINDOW;
    buf[pos + 1] = window_open as u8;
    pos += 2;

    pos
}

/// Encode a heartbeat payload (periodic environmental report).
/// Returns the number of bytes written to `buf`.
#[allow(clippy::too_many_arguments)]
pub fn encode_heartbeat(
    buf: &mut [u8],
    packet_id: u8,
    battery_pct: u8,
    temp_cdeg: i16,
    humidity_cpct: u16,
    window_open: bool,
    is_tilt: bool,
    tampered: bool,
    problem: bool,
) -> usize {
    let mut pos = 0;

    buf[pos] = DEVICE_INFO;
    pos += 1;

    buf[pos] = obj_id::PACKET_ID;
    buf[pos + 1] = packet_id;
    pos += 2;

    buf[pos] = obj_id::BATTERY;
    buf[pos + 1] = battery_pct;
    pos += 2;

    // Temperature: sint16 LE, factor 0.01°C
    buf[pos] = obj_id::TEMPERATURE;
    let t_bytes = temp_cdeg.to_le_bytes();
    buf[pos + 1] = t_bytes[0];
    buf[pos + 2] = t_bytes[1];
    pos += 3;

    // Humidity: uint16 LE, factor 0.01%
    buf[pos] = obj_id::HUMIDITY;
    let h_bytes = humidity_cpct.to_le_bytes();
    buf[pos + 1] = h_bytes[0];
    buf[pos + 2] = h_bytes[1];
    pos += 3;

    buf[pos] = obj_id::GENERIC_BOOLEAN;
    buf[pos + 1] = is_tilt as u8;
    pos += 2;

    buf[pos] = obj_id::PROBLEM;
    buf[pos + 1] = problem as u8;
    pos += 2;

    buf[pos] = obj_id::TAMPER;
    buf[pos + 1] = tampered as u8;
    pos += 2;

    buf[pos] = obj_id::WINDOW;
    buf[pos + 1] = window_open as u8;
    pos += 2;

    pos
}

pub fn encode_button_event(buf: &mut [u8], packet_id: u8, event: ButtonEvent) -> usize {
    let mut pos = 0;

    buf[pos] = DEVICE_INFO;
    pos += 1;

    buf[pos] = obj_id::PACKET_ID;
    buf[pos + 1] = packet_id;
    pos += 2;

    buf[pos] = obj_id::BUTTON_EVENT;
    buf[pos + 1] = event.event_id();
    pos += 2;

    pos
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_state_event_closed() {
        let mut buf = [0u8; 32];
        let len = encode_state_event(&mut buf, 0, false, false, false, false);
        assert_eq!(len, 11);
        assert_eq!(buf[0], DEVICE_INFO);
        assert_eq!(buf[1], obj_id::PACKET_ID);
        assert_eq!(buf[2], 0); // packet_id = 0
        assert_eq!(buf[3], obj_id::GENERIC_BOOLEAN);
        assert_eq!(buf[4], 0); // not tilt
        assert_eq!(buf[5], obj_id::PROBLEM);
        assert_eq!(buf[6], 0); // no problem
        assert_eq!(buf[7], obj_id::TAMPER);
        assert_eq!(buf[8], 0); // not tampered
        assert_eq!(buf[9], obj_id::WINDOW);
        assert_eq!(buf[10], 0); // closed
    }

    #[test]
    fn test_encode_state_event_tilt() {
        let mut buf = [0u8; 32];
        let len = encode_state_event(&mut buf, 5, true, true, false, false);
        assert_eq!(len, 11);
        assert_eq!(buf[2], 5); // packet_id
        assert_eq!(buf[4], 1); // tilt
        assert_eq!(buf[6], 0); // no problem
        assert_eq!(buf[8], 0); // not tampered
        assert_eq!(buf[10], 1); // open
    }

    #[test]
    fn test_encode_state_event_tampered() {
        let mut buf = [0u8; 32];
        let len = encode_state_event(&mut buf, 255, true, false, true, true);
        assert_eq!(len, 11);
        assert_eq!(buf[2], 255);
        assert_eq!(buf[4], 0); // not tilt
        assert_eq!(buf[6], 1); // problem
        assert_eq!(buf[8], 1); // tampered
        assert_eq!(buf[10], 1); // open
    }

    #[test]
    fn test_encode_heartbeat_basic() {
        let mut buf = [0u8; 32];
        let len = encode_heartbeat(&mut buf, 42, 75, 2350, 6520, false, false, false, false);
        assert_eq!(buf[0], DEVICE_INFO);
        // Battery = 75%
        assert_eq!(buf[3], obj_id::BATTERY);
        assert_eq!(buf[4], 75);
        // Temperature = 2350 centidegrees = 23.50°C
        assert_eq!(buf[5], obj_id::TEMPERATURE);
        assert_eq!(i16::from_le_bytes([buf[6], buf[7]]), 2350);
        // Humidity = 6520 centipercent = 65.20%
        assert_eq!(buf[8], obj_id::HUMIDITY);
        assert_eq!(u16::from_le_bytes([buf[9], buf[10]]), 6520);
        assert_eq!(buf[11], obj_id::GENERIC_BOOLEAN);
        assert_eq!(buf[12], 0);
        assert_eq!(buf[13], obj_id::PROBLEM);
        assert_eq!(buf[14], 0);
        assert_eq!(buf[15], obj_id::TAMPER);
        assert_eq!(buf[16], 0);
        assert_eq!(buf[17], obj_id::WINDOW);
        assert_eq!(buf[18], 0);
        // Must fit in standard BLE advert
        assert!(len <= 27, "Heartbeat {} bytes exceeds 27-byte limit", len);
    }

    #[test]
    fn test_heartbeat_max_values_fits_ble() {
        let mut buf = [0u8; 32];
        let len = encode_heartbeat(&mut buf, 255, 100, 8500, 10000, true, true, true, true);
        // AD structure: len(1) + type(1) + UUID(2) + payload ≤ 27
        assert!(len <= 27, "Max heartbeat {} bytes exceeds limit", len);
    }

    #[test]
    fn test_heartbeat_negative_temperature() {
        let mut buf = [0u8; 32];
        let _len = encode_heartbeat(&mut buf, 1, 50, -1500, 3000, false, false, false, false);
        // -15.00°C encoded as sint16 LE
        assert_eq!(i16::from_le_bytes([buf[6], buf[7]]), -1500);
    }

    #[test]
    fn test_device_info_byte() {
        // Version 2 (bits 7-5 = 010), trigger-based (bit 2 = 1)
        assert_eq!(DEVICE_INFO & 0xE0, 0x40); // version = 2
        assert_eq!(DEVICE_INFO & 0x04, 0x04); // trigger-based
        assert_eq!(DEVICE_INFO & 0x01, 0x00); // not encrypted
    }

    #[test]
    fn test_encode_button_event_double_press() {
        let mut buf = [0u8; 32];
        let len = encode_button_event(&mut buf, 9, ButtonEvent::DoublePress);
        assert_eq!(len, 5);
        assert_eq!(buf[0], DEVICE_INFO);
        assert_eq!(buf[1], obj_id::PACKET_ID);
        assert_eq!(buf[2], 9);
        assert_eq!(buf[3], obj_id::BUTTON_EVENT);
        assert_eq!(buf[4], 0x02);
    }
}
