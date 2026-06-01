//! MEMS-driven button/event detection from knock and sustained motion patterns.

use crate::bthome::ButtonEvent;

const MAX_KNOCKS: usize = 3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), derive(defmt::Format))]
pub struct MemsButtonConfig {
    pub multi_knock_window_ms: u64,
    pub finalize_quiet_ms: u64,
    pub long_press_ms: u64,
    pub hold_press_ms: u64,
}

impl Default for MemsButtonConfig {
    fn default() -> Self {
        Self {
            multi_knock_window_ms: 900,
            finalize_quiet_ms: 700,
            long_press_ms: 1_500,
            hold_press_ms: 2_500,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MemsButtonSignal {
    Knock,
    MotionStart,
    MotionEnd,
    Tick,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MemsButtonDetector {
    config: MemsButtonConfig,
    knocks_ms: [u64; MAX_KNOCKS],
    knock_len: usize,
    motion_start_ms: Option<u64>,
    hold_reported: bool,
}

impl MemsButtonDetector {
    pub const fn new(config: MemsButtonConfig) -> Self {
        Self {
            config,
            knocks_ms: [0; MAX_KNOCKS],
            knock_len: 0,
            motion_start_ms: None,
            hold_reported: false,
        }
    }

    pub fn record(&mut self, signal: MemsButtonSignal, at_ms: u64) -> Option<ButtonEvent> {
        match signal {
            MemsButtonSignal::Knock => {
                self.prune_knocks(at_ms);
                self.push_knock(at_ms);
                None
            }
            MemsButtonSignal::MotionStart => {
                self.motion_start_ms = Some(at_ms);
                self.hold_reported = false;
                None
            }
            MemsButtonSignal::MotionEnd => self.finish_motion(at_ms),
            MemsButtonSignal::Tick => {
                if let Some(event) = self.maybe_emit_hold(at_ms) {
                    return Some(event);
                }
                self.finalize_knocks(at_ms)
            }
        }
    }

    pub fn next_deadline_ms(&self, _now_ms: u64) -> Option<u64> {
        let knock_deadline = if self.knock_len > 0 {
            Some(self.knocks_ms[self.knock_len - 1] + self.config.finalize_quiet_ms)
        } else {
            None
        };

        let hold_deadline = match self.motion_start_ms {
            Some(started) if !self.hold_reported => Some(started + self.config.hold_press_ms),
            _ => None,
        };

        match (knock_deadline, hold_deadline) {
            (Some(knock), Some(hold)) => Some(knock.min(hold)),
            (Some(knock), None) => Some(knock),
            (None, Some(hold)) => Some(hold),
            (None, None) => None,
        }
    }

    fn push_knock(&mut self, at_ms: u64) {
        if self.knock_len < MAX_KNOCKS {
            self.knocks_ms[self.knock_len] = at_ms;
            self.knock_len += 1;
            return;
        }

        self.knocks_ms.copy_within(1..MAX_KNOCKS, 0);
        self.knocks_ms[MAX_KNOCKS - 1] = at_ms;
    }

    fn prune_knocks(&mut self, at_ms: u64) {
        let oldest_allowed = at_ms.saturating_sub(self.config.multi_knock_window_ms);
        let first_valid = self.knocks_ms[..self.knock_len]
            .iter()
            .position(|&ts| ts >= oldest_allowed)
            .unwrap_or(self.knock_len);

        if first_valid >= self.knock_len {
            self.knock_len = 0;
            return;
        }

        if first_valid > 0 {
            self.knocks_ms.copy_within(first_valid..self.knock_len, 0);
            self.knock_len -= first_valid;
        }
    }

    fn finalize_knocks(&mut self, at_ms: u64) -> Option<ButtonEvent> {
        if self.knock_len == 0 {
            return None;
        }

        let newest = self.knocks_ms[self.knock_len - 1];
        if at_ms.saturating_sub(newest) < self.config.finalize_quiet_ms {
            return None;
        }

        let count = self.knock_len;
        self.knock_len = 0;

        match count {
            1 => Some(ButtonEvent::Press),
            2 => Some(ButtonEvent::DoublePress),
            _ => Some(ButtonEvent::TriplePress),
        }
    }

    fn maybe_emit_hold(&mut self, at_ms: u64) -> Option<ButtonEvent> {
        let started = self.motion_start_ms?;
        if self.hold_reported || at_ms.saturating_sub(started) < self.config.hold_press_ms {
            return None;
        }

        self.hold_reported = true;
        Some(ButtonEvent::HoldPress)
    }

    fn finish_motion(&mut self, at_ms: u64) -> Option<ButtonEvent> {
        let started = self.motion_start_ms.take()?;
        let duration_ms = at_ms.saturating_sub(started);

        if self.hold_reported {
            self.hold_reported = false;
            return None;
        }

        if duration_ms >= self.config.long_press_ms {
            return Some(ButtonEvent::LongPress);
        }

        None
    }
}

impl Default for MemsButtonDetector {
    fn default() -> Self {
        Self::new(MemsButtonConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn single_knock_becomes_press_after_quiet_gap() {
        let mut detector = MemsButtonDetector::default();

        assert_eq!(detector.record(MemsButtonSignal::Knock, 0), None);
        assert_eq!(detector.record(MemsButtonSignal::Tick, 500), None);
        assert_eq!(
            detector.record(MemsButtonSignal::Tick, 800),
            Some(ButtonEvent::Press)
        );
    }

    #[test]
    fn double_knock_becomes_double_press() {
        let mut detector = MemsButtonDetector::default();

        assert_eq!(detector.record(MemsButtonSignal::Knock, 0), None);
        assert_eq!(detector.record(MemsButtonSignal::Knock, 300), None);
        assert_eq!(
            detector.record(MemsButtonSignal::Tick, 1_100),
            Some(ButtonEvent::DoublePress)
        );
    }

    #[test]
    fn triple_knock_caps_to_triple_press() {
        let mut detector = MemsButtonDetector::default();

        assert_eq!(detector.record(MemsButtonSignal::Knock, 0), None);
        assert_eq!(detector.record(MemsButtonSignal::Knock, 200), None);
        assert_eq!(detector.record(MemsButtonSignal::Knock, 400), None);
        assert_eq!(
            detector.record(MemsButtonSignal::Tick, 1_200),
            Some(ButtonEvent::TriplePress)
        );
    }

    #[test]
    fn slow_knocks_do_not_merge() {
        let mut detector = MemsButtonDetector::default();

        assert_eq!(detector.record(MemsButtonSignal::Knock, 0), None);
        assert_eq!(
            detector.record(MemsButtonSignal::Tick, 1_000),
            Some(ButtonEvent::Press)
        );
        assert_eq!(detector.record(MemsButtonSignal::Knock, 2_000), None);
        assert_eq!(
            detector.record(MemsButtonSignal::Tick, 2_800),
            Some(ButtonEvent::Press)
        );
    }

    #[test]
    fn long_motion_release_becomes_long_press() {
        let mut detector = MemsButtonDetector::default();

        assert_eq!(detector.record(MemsButtonSignal::MotionStart, 0), None);
        assert_eq!(
            detector.record(MemsButtonSignal::MotionEnd, 1_800),
            Some(ButtonEvent::LongPress)
        );
    }

    #[test]
    fn long_hold_emits_hold_press_once() {
        let mut detector = MemsButtonDetector::default();

        assert_eq!(detector.record(MemsButtonSignal::MotionStart, 0), None);
        assert_eq!(detector.record(MemsButtonSignal::Tick, 2_000), None);
        assert_eq!(
            detector.record(MemsButtonSignal::Tick, 2_600),
            Some(ButtonEvent::HoldPress)
        );
        assert_eq!(detector.record(MemsButtonSignal::Tick, 3_000), None);
        assert_eq!(detector.record(MemsButtonSignal::MotionEnd, 3_200), None);
    }
}
