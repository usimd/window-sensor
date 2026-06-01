//! Single place to tune MEMS sensitivity without touching the detection code.

use crate::mems::{
    IdsConfig, IdsSensitivityProfile, MemsConfig, MemsInterruptConfig, MemsOrientationThreshold,
};
use crate::mems_button::MemsButtonConfig;

/// Default operational IDS sensitivity profile.
pub const IDS_PROFILE: IdsSensitivityProfile = IdsSensitivityProfile::Balanced;

/// Raw LIS2DTW12 interrupt tuning.
pub const INTERRUPT_CONFIG: MemsInterruptConfig = MemsInterruptConfig {
    wake_up_threshold: 2,
    wake_up_duration: 1,
    sleep_mode_enabled: true,
    orientation_detection_enabled: true,
    orientation_threshold: MemsOrientationThreshold::Deg70,
    tap_detection_enabled: true,
    tap_threshold_x: 9,
    tap_threshold_y: 9,
    tap_threshold_z: 12,
    tap_quiet_time: 1,
    tap_shock_time: 2,
    double_tap_latency: 6,
};

/// Higher-level knock/button grouping.
pub const BUTTON_CONFIG: MemsButtonConfig = MemsButtonConfig {
    multi_knock_window_ms: 900,
    finalize_quiet_ms: 700,
    long_press_ms: 1_500,
    hold_press_ms: 2_500,
};

pub const fn mems_config() -> MemsConfig {
    MemsConfig {
        interrupts: INTERRUPT_CONFIG,
        ids: IdsConfig::for_profile(IDS_PROFILE),
        button: BUTTON_CONFIG,
    }
}
