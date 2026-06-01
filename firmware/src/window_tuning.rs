//! Single place to tune window detection and calibration behavior.

use crate::classifier::Thresholds;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WindowCalibration {
    pub closed_mt: f32,
    pub tilt_mt: f32,
    pub open_mt: f32,
}

/// Fallback closed-field baseline before installation calibration exists.
pub const DEFAULT_CLOSED_BASELINE_MT: f32 = 50.0;

/// TMAG wake threshold sits this far below the calibrated closed field.
pub const INT_TRIGGER_MARGIN_MT: f32 = 5.0;

/// A field above this derived threshold is treated as definitely closed.
pub const CLOSED_RETURN_MARGIN_MT: f32 = 10.0;

/// Reject calibration captures outside a plausible installed closed-field range.
pub const MIN_VALID_CLOSED_BASELINE_MT: f32 = 20.0;
pub const MAX_VALID_CLOSED_BASELINE_MT: f32 = 75.0;
pub const MAX_VALID_TILT_BASELINE_MT: f32 = 70.0;
pub const MAX_VALID_OPEN_BASELINE_MT: f32 = 40.0;
pub const MIN_CLOSED_TO_TILT_DELTA_MT: f32 = 8.0;
pub const MIN_TILT_TO_OPEN_DELTA_MT: f32 = 2.0;

/// Burst capture timing after a TMAG interrupt.
pub const BURST_SAMPLES: usize = 12;
pub const BURST_INTERVAL_MS: u64 = 50;
pub const REARM_DELAY_MS: u64 = 200;

/// Closed-position calibration capture timing.
pub const CALIBRATION_SAMPLES: usize = 8;
pub const CALIBRATION_INTERVAL_MS: u64 = 25;
pub const CALIBRATION_MAX_ATTEMPTS: usize = 6;
pub const CALIBRATION_MAX_SPREAD_MT: f32 = 2.0;

/// Classifier tuning for OPEN vs TILT.
pub const CLASSIFIER_THRESHOLDS: Thresholds = Thresholds {
    collapse_threshold_mt: 35.0,
    min_samples_for_open: 5,
    noise_floor_mt: 0.5,
};

/// Lower clamp keeps thresholds meaningful even for weak installs.
pub const MIN_INT_THRESHOLD_MT: f32 = 5.0;
pub const MIN_CLOSED_THRESHOLD_MT: f32 = 5.0;

pub fn wake_threshold_mt(calibration: Option<WindowCalibration>) -> f32 {
    match calibration {
        Some(calibration) => clamp_min(
            midpoint(calibration.closed_mt, calibration.tilt_mt),
            MIN_INT_THRESHOLD_MT,
        ),
        None => clamp_min(
            DEFAULT_CLOSED_BASELINE_MT - INT_TRIGGER_MARGIN_MT,
            MIN_INT_THRESHOLD_MT,
        ),
    }
}

pub fn closed_threshold_mt(calibration: Option<WindowCalibration>) -> f32 {
    match calibration {
        Some(calibration) => clamp_min(
            calibration.closed_mt
                - CLOSED_RETURN_MARGIN_MT.min((calibration.closed_mt - calibration.tilt_mt) / 2.0),
            MIN_CLOSED_THRESHOLD_MT,
        ),
        None => clamp_min(
            DEFAULT_CLOSED_BASELINE_MT - CLOSED_RETURN_MARGIN_MT,
            MIN_CLOSED_THRESHOLD_MT,
        ),
    }
}

pub fn is_valid_closed_baseline_mt(value_mt: f32) -> bool {
    (MIN_VALID_CLOSED_BASELINE_MT..=MAX_VALID_CLOSED_BASELINE_MT).contains(&value_mt)
}

pub fn is_valid_window_calibration(calibration: WindowCalibration) -> bool {
    is_valid_closed_baseline_mt(calibration.closed_mt)
        && calibration.tilt_mt >= 0.0
        && calibration.tilt_mt <= MAX_VALID_TILT_BASELINE_MT
        && calibration.open_mt >= 0.0
        && calibration.open_mt <= MAX_VALID_OPEN_BASELINE_MT
        && calibration.closed_mt > calibration.tilt_mt
        && calibration.tilt_mt > calibration.open_mt
        && (calibration.closed_mt - calibration.tilt_mt) >= MIN_CLOSED_TO_TILT_DELTA_MT
        && (calibration.tilt_mt - calibration.open_mt) >= MIN_TILT_TO_OPEN_DELTA_MT
}

pub fn is_stable_calibration_window(min_mt: f32, max_mt: f32) -> bool {
    (max_mt - min_mt) <= CALIBRATION_MAX_SPREAD_MT
}

const fn clamp_min(value: f32, min: f32) -> f32 {
    if value < min { min } else { value }
}

fn midpoint(a: f32, b: f32) -> f32 {
    (a + b) / 2.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thresholds_fall_back_to_default_baseline() {
        assert_eq!(wake_threshold_mt(None), 45.0);
        assert_eq!(closed_threshold_mt(None), 40.0);
    }

    #[test]
    fn thresholds_follow_calibrated_closed_field() {
        let calibration = WindowCalibration {
            closed_mt: 62.0,
            tilt_mt: 26.0,
            open_mt: 6.0,
        };

        assert_eq!(wake_threshold_mt(Some(calibration)), 44.0);
        assert_eq!(closed_threshold_mt(Some(calibration)), 52.0);
    }

    #[test]
    fn baseline_validation_rejects_implausible_values() {
        assert!(!is_valid_closed_baseline_mt(10.0));
        assert!(is_valid_closed_baseline_mt(50.0));
        assert!(!is_valid_closed_baseline_mt(90.0));
    }

    #[test]
    fn full_calibration_requires_ordered_span() {
        assert!(is_valid_window_calibration(WindowCalibration {
            closed_mt: 55.0,
            tilt_mt: 25.0,
            open_mt: 5.0,
        }));
        assert!(!is_valid_window_calibration(WindowCalibration {
            closed_mt: 55.0,
            tilt_mt: 50.0,
            open_mt: 5.0,
        }));
    }

    #[test]
    fn calibration_window_requires_small_spread() {
        assert!(is_stable_calibration_window(24.5, 26.4));
        assert!(!is_stable_calibration_window(24.5, 27.0));
    }
}
