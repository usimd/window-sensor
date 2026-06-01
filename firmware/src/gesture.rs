//! Magnet gesture recognition from window state transitions.

/// User-triggered maintenance modes encoded as rapid open/close cycles.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), derive(defmt::Format))]
pub enum Gesture {
    Calibrate,
    Pairing,
    FactoryReset,
}

const MAX_TRANSITIONS: usize = 14;
const PAIRING_WINDOW_MS: u64 = 10_000;
const FACTORY_RESET_WINDOW_MS: u64 = 10_000;
const CALIBRATE_WINDOW_MS: u64 = 5_000;
const RETENTION_WINDOW_MS: u64 = 11_000;
const CALIBRATE_TRANSITIONS: usize = 6;
const PAIRING_TRANSITIONS: usize = 10;
const FACTORY_RESET_TRANSITIONS: usize = 14;

/// Tracks the most recent state transitions without heap allocation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GestureDetector {
    transitions_ms: [u64; MAX_TRANSITIONS],
    len: usize,
    pending: Option<Gesture>,
}

impl GestureDetector {
    pub const fn new() -> Self {
        Self {
            transitions_ms: [0; MAX_TRANSITIONS],
            len: 0,
            pending: None,
        }
    }

    /// Record a single window transition timestamp in milliseconds since boot.
    /// Smaller gestures are held as pending so they can escalate into larger
    /// ones if more transitions arrive within the allowed time window.
    pub fn record_transition(&mut self, at_ms: u64) -> Option<Gesture> {
        self.push(at_ms);
        self.prune_older_than(at_ms.saturating_sub(RETENTION_WINDOW_MS));

        if self
            .window_for_last(FACTORY_RESET_TRANSITIONS)
            .is_some_and(|w| w <= FACTORY_RESET_WINDOW_MS)
        {
            self.clear();
            return Some(Gesture::FactoryReset);
        }

        if self
            .window_for_last(PAIRING_TRANSITIONS)
            .is_some_and(|w| w <= PAIRING_WINDOW_MS)
        {
            self.pending = Some(Gesture::Pairing);
            return None;
        }

        if self
            .window_for_last(CALIBRATE_TRANSITIONS)
            .is_some_and(|w| w <= CALIBRATE_WINDOW_MS)
        {
            self.pending = Some(Gesture::Calibrate);
            return None;
        }

        let pending = self.pending?;
        let newest = self.transitions_ms[self.len - 1];
        let oldest = self.transitions_ms[0];
        let timeout_ms = match pending {
            Gesture::Calibrate => CALIBRATE_WINDOW_MS,
            Gesture::Pairing => PAIRING_WINDOW_MS,
            Gesture::FactoryReset => FACTORY_RESET_WINDOW_MS,
        };

        if newest.saturating_sub(oldest) > timeout_ms {
            self.clear();
            return Some(pending);
        }

        None
    }

    fn push(&mut self, at_ms: u64) {
        if self.len < MAX_TRANSITIONS {
            self.transitions_ms[self.len] = at_ms;
            self.len += 1;
            return;
        }

        self.transitions_ms.copy_within(1..MAX_TRANSITIONS, 0);
        self.transitions_ms[MAX_TRANSITIONS - 1] = at_ms;
    }

    fn prune_older_than(&mut self, oldest_allowed_ms: u64) {
        let first_valid = self.transitions_ms[..self.len]
            .iter()
            .position(|&ts| ts >= oldest_allowed_ms)
            .unwrap_or(self.len);

        if first_valid == 0 {
            return;
        }

        if first_valid >= self.len {
            self.clear();
            return;
        }

        self.transitions_ms.copy_within(first_valid..self.len, 0);
        self.len -= first_valid;
    }

    fn window_for_last(&self, count: usize) -> Option<u64> {
        if self.len < count {
            return None;
        }

        let start = self.transitions_ms[self.len - count];
        let end = self.transitions_ms[self.len - 1];
        Some(end.saturating_sub(start))
    }

    fn clear(&mut self) {
        self.len = 0;
        self.pending = None;
    }
}

impl Default for GestureDetector {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detects_calibrate_gesture() {
        let mut detector = GestureDetector::new();
        let mut result = None;

        for ts in [0, 800, 1600, 2400, 3200, 4000] {
            result = detector.record_transition(ts);
        }

        assert_eq!(result, None);
        assert_eq!(detector.record_transition(9_200), Some(Gesture::Calibrate));
    }

    #[test]
    fn detects_pairing_gesture() {
        let mut detector = GestureDetector::new();
        let mut result = None;

        for ts in [0, 900, 1800, 2700, 3600, 4500, 5400, 6300, 7200, 8100] {
            result = detector.record_transition(ts);
        }

        assert_eq!(result, None);
        assert_eq!(detector.record_transition(19_000), Some(Gesture::Pairing));
    }

    #[test]
    fn detects_factory_reset_gesture() {
        let mut detector = GestureDetector::new();
        let mut result = None;

        for ts in [
            0, 650, 1300, 1950, 2600, 3250, 3900, 4550, 5200, 5850, 6500, 7150, 7800, 8450,
        ] {
            result = detector.record_transition(ts);
        }

        assert_eq!(result, Some(Gesture::FactoryReset));
    }

    #[test]
    fn ignores_slow_transitions() {
        let mut detector = GestureDetector::new();
        let mut result = None;

        for ts in [0, 1500, 3000, 4500, 6000, 7500] {
            result = detector.record_transition(ts);
        }

        assert_eq!(result, None);
    }

    #[test]
    fn detector_resets_after_trigger() {
        let mut detector = GestureDetector::new();

        for ts in [0, 800, 1600, 2400, 3200, 4000] {
            let _ = detector.record_transition(ts);
        }

        assert_eq!(detector.record_transition(9_200), Some(Gesture::Calibrate));
        assert_eq!(detector.record_transition(20_000), None);
        assert_eq!(detector.record_transition(20_800), None);
    }
}
