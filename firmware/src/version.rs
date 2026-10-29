//! Firmware version as advertised over BTHome (object 0xF2) and reported to OTA clients.
//!
//! Release builds set `WINDOW_SENSOR_VERSION` from the git tag (`v1.2.3` -> `1.2.3`);
//! local builds fall back to the crate version.

pub const VERSION_STR: &str = match option_env!("WINDOW_SENSOR_VERSION") {
    Some(v) => v,
    None => env!("CARGO_PKG_VERSION"),
};

/// `[major, minor, patch]`; anything after the patch number (pre-release, build) is ignored.
pub const FIRMWARE_VERSION: [u8; 3] = parse_semver(VERSION_STR);

pub const fn parse_semver(s: &str) -> [u8; 3] {
    let bytes = s.as_bytes();
    let mut parts = [0u16; 3];
    let mut part = 0;
    let mut i = 0;
    while i < bytes.len() && part < 3 {
        let c = bytes[i];
        if c.is_ascii_digit() {
            parts[part] = parts[part] * 10 + (c - b'0') as u16;
            assert!(
                parts[part] <= u8::MAX as u16,
                "version component exceeds 255"
            );
        } else if c == b'.' {
            part += 1;
        } else {
            break;
        }
        i += 1;
    }
    [parts[0] as u8, parts[1] as u8, parts[2] as u8]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_release_version() {
        assert_eq!(parse_semver("1.2.3"), [1, 2, 3]);
    }

    #[test]
    fn ignores_build_and_prerelease_suffix() {
        assert_eq!(parse_semver("0.0.0+42"), [0, 0, 0]);
        assert_eq!(parse_semver("2.10.7-rc.1"), [2, 10, 7]);
    }

    #[test]
    fn missing_components_are_zero() {
        assert_eq!(parse_semver("3"), [3, 0, 0]);
    }
}
