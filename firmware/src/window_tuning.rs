//! Single place to tune window detection and calibration behavior.

use crate::classifier::Thresholds;

/// Fallback closed-field baseline before installation calibration exists.
pub const DEFAULT_CLOSED_BASELINE_MT: f32 = 50.0;

/// TMAG wake threshold sits this far below the calibrated closed field.
pub const INT_TRIGGER_MARGIN_MT: f32 = 5.0;

/// A field above this derived threshold is treated as definitely closed.
pub const CLOSED_RETURN_MARGIN_MT: f32 = 10.0;

/// Reject calibration captures outside a plausible installed closed-field range.
pub const MIN_VALID_CLOSED_BASELINE_MT: f32 = 20.0;
pub const MAX_VALID_CLOSED_BASELINE_MT: f32 = 75.0;

/// Burst capture timing after a TMAG interrupt.
pub const BURST_SAMPLES: usize = 12;
pub const BURST_INTERVAL_MS: u64 = 50;
pub const REARM_DELAY_MS: u64 = 200;

/// Closed-position calibration capture timing.
pub const CALIBRATION_SAMPLES: usize = 8;
pub const CALIBRATION_INTERVAL_MS: u64 = 25;

/// Classifier tuning for OPEN vs TILT.
pub const CLASSIFIER_THRESHOLDS: Thresholds = Thresholds {
    collapse_threshold_mt: 35.0,
    min_samples_for_open: 5,
    noise_floor_mt: 0.5,
};

/// Lower clamp keeps thresholds meaningful even for weak installs.
pub const MIN_INT_THRESHOLD_MT: f32 = 5.0;
pub const MIN_CLOSED_THRESHOLD_MT: f32 = 5.0;

pub fn wake_threshold_mt(calibrated_closed_baseline_mt: Option<f32>) -> f32 {
    let baseline = calibrated_closed_baseline_mt.unwrap_or(DEFAULT_CLOSED_BASELINE_MT);
    clamp_min(baseline - INT_TRIGGER_MARGIN_MT, MIN_INT_THRESHOLD_MT)
}

pub fn closed_threshold_mt(calibrated_closed_baseline_mt: Option<f32>) -> f32 {
    let baseline = calibrated_closed_baseline_mt.unwrap_or(DEFAULT_CLOSED_BASELINE_MT);
    clamp_min(baseline - CLOSED_RETURN_MARGIN_MT, MIN_CLOSED_THRESHOLD_MT)
}

pub fn is_valid_closed_baseline_mt(value_mt: f32) -> bool {
    (MIN_VALID_CLOSED_BASELINE_MT..=MAX_VALID_CLOSED_BASELINE_MT).contains(&value_mt)
}

const fn clamp_min(value: f32, min: f32) -> f32 {
    if value < min { min } else { value }
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
        assert_eq!(wake_threshold_mt(Some(62.0)), 57.0);
        assert_eq!(closed_threshold_mt(Some(62.0)), 52.0);
    }

    #[test]
    fn baseline_validation_rejects_implausible_values() {
        assert!(!is_valid_closed_baseline_mt(10.0));
        assert!(is_valid_closed_baseline_mt(50.0));
        assert!(!is_valid_closed_baseline_mt(90.0));
    }
}
