//! Window state classifier — pure logic, no hardware dependencies.
//!
//! Classifies magnetic field burst data into CLOSED / TILT / OPEN states.
//! All math is f32 (nRF54L10 Cortex-M33 has hardware FPU).

/// Window state (3-state output)
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), derive(defmt::Format))]
pub enum WindowState {
    Closed,
    Tilt,
    Open,
}

/// A single magnetic field reading (raw 12-bit per axis, ±80 mT range)
#[derive(Clone, Copy, Debug)]
pub struct MagSample {
    pub x_raw: i16,
    pub y_raw: i16,
    pub z_raw: i16,
}

impl MagSample {
    /// Sensitivity for TMAG5273A2 ±80 mT range: 80000 µT / 2048 ≈ 39.0625 µT/LSB
    const UT_PER_LSB: f32 = 39.0625;

    pub fn x_mt(&self) -> f32 {
        self.x_raw as f32 * Self::UT_PER_LSB / 1000.0
    }

    pub fn y_mt(&self) -> f32 {
        self.y_raw as f32 * Self::UT_PER_LSB / 1000.0
    }

    pub fn z_mt(&self) -> f32 {
        self.z_raw as f32 * Self::UT_PER_LSB / 1000.0
    }

    /// Magnitude in mT (euclidean norm)
    pub fn magnitude_mt(&self) -> f32 {
        let x = self.x_mt();
        let y = self.y_mt();
        let z = self.z_mt();
        libm::sqrtf(x * x + y * y + z * z)
    }
}

/// Classification thresholds (derived from TIMSS simulation data)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Thresholds {
    /// If magnitude drops by more than this in ≤2 samples → TILT
    pub collapse_threshold_mt: f32,
    /// Minimum number of samples above noise floor to classify as OPEN
    pub min_samples_for_open: usize,
    /// Noise floor — readings below this are considered "field gone"
    pub noise_floor_mt: f32,
}

impl Default for Thresholds {
    fn default() -> Self {
        Self {
            collapse_threshold_mt: 35.0,
            min_samples_for_open: 5,
            noise_floor_mt: 0.5,
        }
    }
}

/// Classify a burst of samples taken at 20 Hz after the TMAG5273 INT fires.
///
/// The burst is captured immediately after the magnetic field drops below
/// the INT threshold (~45 mT), indicating the window is moving.
///
/// Classification logic:
/// - If field collapses by >35 mT in ≤2 samples (0.1 s) → TILT
/// - If field decays gradually with ≥5 samples above noise → OPEN
/// - Empty/ambiguous → OPEN (conservative assumption)
pub fn classify_burst(burst: &[MagSample], thresholds: &Thresholds) -> WindowState {
    if burst.is_empty() {
        return WindowState::Open;
    }

    let mag0 = burst[0].magnitude_mt();

    // Check for instantaneous collapse (TILT pattern)
    let check_count = burst.len().min(3);
    for sample in &burst[1..check_count] {
        let drop = mag0 - sample.magnitude_mt();
        if drop > thresholds.collapse_threshold_mt {
            return WindowState::Tilt;
        }
    }

    // Count samples with field above noise floor (gradual decay = OPEN)
    let above_noise = burst
        .iter()
        .filter(|s| s.magnitude_mt() > thresholds.noise_floor_mt)
        .count();

    if above_noise >= thresholds.min_samples_for_open {
        WindowState::Open
    } else {
        // Ambiguous: not enough samples above noise, but no collapse detected.
        // Could be very fast open or sensor anomaly — conservatively report Closed.
        WindowState::Closed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(x: i16, y: i16, z: i16) -> MagSample {
        MagSample {
            x_raw: x,
            y_raw: y,
            z_raw: z,
        }
    }

    /// Helper: raw value for a given mT (single axis, others zero)
    fn raw_for_mt(mt: f32) -> i16 {
        (mt * 1000.0 / MagSample::UT_PER_LSB) as i16
    }

    #[test]
    fn test_magnitude_single_axis() {
        let s = sample(raw_for_mt(50.0), 0, 0);
        assert!((s.magnitude_mt() - 50.0).abs() < 0.1);
    }

    #[test]
    fn test_magnitude_zero() {
        let s = sample(0, 0, 0);
        assert_eq!(s.magnitude_mt(), 0.0);
    }

    #[test]
    fn test_magnitude_all_axes() {
        let v = raw_for_mt(10.0);
        let s = sample(v, v, v);
        let expected = libm::sqrtf(3.0) * 10.0;
        assert!((s.magnitude_mt() - expected).abs() < 0.2);
    }

    #[test]
    fn test_classify_tilt_instant_collapse() {
        let thresholds = Thresholds::default();
        // Tilt: 50 mT → 10 mT in one sample (drop = 40 mT > 35 mT threshold)
        let burst = [
            sample(raw_for_mt(50.0), 0, 0),
            sample(raw_for_mt(10.0), 0, 0),
            sample(raw_for_mt(5.0), 0, 0),
        ];
        assert_eq!(classify_burst(&burst, &thresholds), WindowState::Tilt);
    }

    #[test]
    fn test_classify_tilt_two_sample_collapse() {
        let thresholds = Thresholds::default();
        // Drop spread over 2 samples: 50 → 30 → 10
        // First drop = 20 mT (not enough), second drop from initial = 40 mT (enough)
        let burst = [
            sample(raw_for_mt(50.0), 0, 0),
            sample(raw_for_mt(30.0), 0, 0), // drop=20 < 35
            sample(raw_for_mt(10.0), 0, 0), // drop=40 > 35 ✓
        ];
        assert_eq!(classify_burst(&burst, &thresholds), WindowState::Tilt);
    }

    #[test]
    fn test_classify_open_gradual_decay() {
        let thresholds = Thresholds::default();
        // Open: gradual decay ~1.4 mT/sample, stays above noise for many samples
        let burst: [MagSample; 10] =
            core::array::from_fn(|i| sample(raw_for_mt(45.0 - i as f32 * 1.4), 0, 0));
        assert_eq!(classify_burst(&burst, &thresholds), WindowState::Open);
    }

    #[test]
    fn test_classify_empty_burst() {
        let thresholds = Thresholds::default();
        assert_eq!(classify_burst(&[], &thresholds), WindowState::Open);
    }

    #[test]
    fn test_classify_single_sample() {
        let thresholds = Thresholds::default();
        // Only 1 sample: can't determine pattern → CLOSED (ambiguous)
        let burst = [sample(raw_for_mt(45.0), 0, 0)];
        assert_eq!(classify_burst(&burst, &thresholds), WindowState::Closed);
    }

    #[test]
    fn test_classify_not_enough_drop_for_tilt() {
        let thresholds = Thresholds::default();
        // Drop of 30 mT < 35 mT threshold → NOT tilt, classify as open
        let burst = [
            sample(raw_for_mt(50.0), 0, 0),
            sample(raw_for_mt(20.0), 0, 0), // drop=30 < 35
            sample(raw_for_mt(15.0), 0, 0),
            sample(raw_for_mt(10.0), 0, 0),
            sample(raw_for_mt(5.0), 0, 0),
            sample(raw_for_mt(3.0), 0, 0),
        ];
        assert_eq!(classify_burst(&burst, &thresholds), WindowState::Open);
    }

    #[test]
    fn test_raw_for_mt_roundtrip() {
        let target = 40.0_f32;
        let raw = raw_for_mt(target);
        let s = sample(raw, 0, 0);
        assert!((s.magnitude_mt() - target).abs() < 0.1);
    }
}
