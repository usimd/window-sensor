//! Central MEMS configuration shared by the host-testable core and the embedded task.
//!
//! Values intentionally mirror the raw LIS2DTW12 driver API today so we can make
//! the production IDS and knock behavior configurable without changing runtime
//! behavior first.

use crate::mems_button::MemsButtonConfig;
use crate::setup::SetupState;

const IDS_EVENT_HISTORY_LEN: usize = 8;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MemsOrientationThreshold {
    Deg50,
    Deg60,
    #[default]
    Deg70,
    Deg80,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum IdsSensitivityProfile {
    Conservative,
    #[default]
    Balanced,
    Aggressive,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IdsConfig {
    pub orientation_change_tamper: bool,
    pub wake_event_count: usize,
    pub wake_window_ms: u64,
    pub tap_event_count: usize,
    pub tap_window_ms: u64,
}

impl IdsConfig {
    pub const fn for_profile(profile: IdsSensitivityProfile) -> Self {
        match profile {
            IdsSensitivityProfile::Conservative => Self {
                orientation_change_tamper: true,
                wake_event_count: 4,
                wake_window_ms: 4_000,
                tap_event_count: 5,
                tap_window_ms: 2_000,
            },
            IdsSensitivityProfile::Balanced => Self {
                orientation_change_tamper: true,
                wake_event_count: 3,
                wake_window_ms: 2_500,
                tap_event_count: 4,
                tap_window_ms: 1_500,
            },
            IdsSensitivityProfile::Aggressive => Self {
                orientation_change_tamper: true,
                wake_event_count: 2,
                wake_window_ms: 1_500,
                tap_event_count: 3,
                tap_window_ms: 1_200,
            },
        }
    }
}

impl Default for IdsConfig {
    fn default() -> Self {
        Self::for_profile(IdsSensitivityProfile::default())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IdsSignal {
    Wake,
    Tap,
    OrientationChange,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IdsTrigger {
    WakeBurst,
    TapBurst,
    OrientationChange,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IdsDetector {
    config: IdsConfig,
    wakes_ms: [u64; IDS_EVENT_HISTORY_LEN],
    wake_len: usize,
    taps_ms: [u64; IDS_EVENT_HISTORY_LEN],
    tap_len: usize,
}

impl IdsDetector {
    pub const fn new(config: IdsConfig) -> Self {
        Self {
            config,
            wakes_ms: [0; IDS_EVENT_HISTORY_LEN],
            wake_len: 0,
            taps_ms: [0; IDS_EVENT_HISTORY_LEN],
            tap_len: 0,
        }
    }

    pub fn record(&mut self, signal: IdsSignal, at_ms: u64) -> Option<IdsTrigger> {
        match signal {
            IdsSignal::Wake => record_recent_trigger(
                &mut self.wakes_ms,
                &mut self.wake_len,
                at_ms,
                self.config.wake_window_ms,
                self.config.wake_event_count,
                IdsTrigger::WakeBurst,
            ),
            IdsSignal::Tap => record_recent_trigger(
                &mut self.taps_ms,
                &mut self.tap_len,
                at_ms,
                self.config.tap_window_ms,
                self.config.tap_event_count,
                IdsTrigger::TapBurst,
            ),
            IdsSignal::OrientationChange => self
                .config
                .orientation_change_tamper
                .then_some(IdsTrigger::OrientationChange),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MemsInterruptConfig {
    pub wake_up_threshold: u8,
    pub wake_up_duration: u8,
    pub sleep_mode_enabled: bool,
    pub orientation_detection_enabled: bool,
    pub orientation_threshold: MemsOrientationThreshold,
    pub tap_detection_enabled: bool,
    pub tap_threshold_x: u8,
    pub tap_threshold_y: u8,
    pub tap_threshold_z: u8,
    pub tap_quiet_time: u8,
    pub tap_shock_time: u8,
    pub double_tap_latency: u8,
}

impl Default for MemsInterruptConfig {
    fn default() -> Self {
        Self {
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
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MemsConfig {
    pub interrupts: MemsInterruptConfig,
    pub ids: IdsConfig,
    pub button: MemsButtonConfig,
}

impl MemsConfig {
    pub fn with_ids_profile(profile: IdsSensitivityProfile) -> Self {
        Self {
            ids: IdsConfig::for_profile(profile),
            ..Self::default()
        }
    }
}

pub const fn ids_enabled_for_setup_state(state: SetupState) -> bool {
    matches!(
        state,
        SetupState::Ready | SetupState::NeedsRecalibration | SetupState::Debug
    )
}

pub const fn setup_gestures_enabled_for_state(state: SetupState) -> bool {
    !ids_enabled_for_setup_state(state)
}

fn push_recent<const N: usize>(buf: &mut [u64; N], len: &mut usize, at_ms: u64, window_ms: u64) {
    let oldest_allowed = at_ms.saturating_sub(window_ms);
    let first_valid = buf[..*len]
        .iter()
        .position(|&ts| ts >= oldest_allowed)
        .unwrap_or(*len);

    if first_valid >= *len {
        *len = 0;
    } else if first_valid > 0 {
        buf.copy_within(first_valid..*len, 0);
        *len -= first_valid;
    }

    if *len < N {
        buf[*len] = at_ms;
        *len += 1;
        return;
    }

    buf.copy_within(1..N, 0);
    buf[N - 1] = at_ms;
}

fn record_recent_trigger<const N: usize>(
    buf: &mut [u64; N],
    len: &mut usize,
    at_ms: u64,
    window_ms: u64,
    threshold: usize,
    trigger: IdsTrigger,
) -> Option<IdsTrigger> {
    if threshold == 0 {
        return None;
    }

    push_recent(buf, len, at_ms, window_ms);
    if *len >= threshold {
        *len = 0;
        return Some(trigger);
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_interrupt_config_matches_current_firmware_tuning() {
        assert_eq!(
            MemsInterruptConfig::default(),
            MemsInterruptConfig {
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
            }
        );
    }

    #[test]
    fn default_mems_config_reuses_button_defaults() {
        let config = MemsConfig::default();

        assert_eq!(config.interrupts, MemsInterruptConfig::default());
        assert_eq!(config.ids, IdsConfig::default());
        assert_eq!(config.button, MemsButtonConfig::default());
    }

    #[test]
    fn balanced_ids_profile_avoids_tamper_on_triple_knock() {
        let mut detector = IdsDetector::new(IdsConfig::default());

        assert_eq!(detector.record(IdsSignal::Tap, 0), None);
        assert_eq!(detector.record(IdsSignal::Tap, 300), None);
        assert_eq!(detector.record(IdsSignal::Tap, 600), None);
        assert_eq!(detector.record(IdsSignal::Tap, 900), Some(IdsTrigger::TapBurst));
    }

    #[test]
    fn slow_motion_does_not_trip_wake_burst_ids() {
        let mut detector = IdsDetector::new(IdsConfig::default());

        assert_eq!(detector.record(IdsSignal::Wake, 0), None);
        assert_eq!(detector.record(IdsSignal::Wake, 3_000), None);
        assert_eq!(detector.record(IdsSignal::Wake, 6_000), None);
    }

    #[test]
    fn orientation_change_can_trigger_immediate_tamper() {
        let mut detector = IdsDetector::new(IdsConfig::default());

        assert_eq!(
            detector.record(IdsSignal::OrientationChange, 123),
            Some(IdsTrigger::OrientationChange)
        );
    }

    #[test]
    fn ids_mode_only_applies_in_operational_states() {
        assert!(!ids_enabled_for_setup_state(SetupState::FactoryNew));
        assert!(!ids_enabled_for_setup_state(SetupState::Discovery));
        assert!(ids_enabled_for_setup_state(SetupState::Ready));
        assert!(ids_enabled_for_setup_state(SetupState::NeedsRecalibration));
        assert!(ids_enabled_for_setup_state(SetupState::Debug));
    }
}
