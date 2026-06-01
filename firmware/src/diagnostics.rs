//! Small host-testable runtime diagnostics latch.

const RESETREAS_DOG0: u32 = 1 << 1;
const RESETREAS_DOG1: u32 = 1 << 2;
const RESETREAS_LOCKUP: u32 = 1 << 7;
const RESETREAS_SECTAMPER: u32 = 1 << 13;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), derive(defmt::Format))]
pub struct DiagnosticsState {
    pub last_resetreas_raw: u32,
    pub unexpected_reset: bool,
    pub pofwarn_seen: bool,
}

impl DiagnosticsState {
    pub const fn new() -> Self {
        Self {
            last_resetreas_raw: 0,
            unexpected_reset: false,
            pofwarn_seen: false,
        }
    }

    pub fn note_resetreas(&mut self, raw: u32) {
        self.last_resetreas_raw = raw;
        self.unexpected_reset = reset_reason_is_problem(raw);
    }

    pub fn note_pofwarn(&mut self) {
        self.pofwarn_seen = true;
    }

    pub const fn problem_active(&self) -> bool {
        self.unexpected_reset || self.pofwarn_seen
    }
}

pub const fn reset_reason_is_problem(raw: u32) -> bool {
    (raw & (RESETREAS_DOG0 | RESETREAS_DOG1 | RESETREAS_LOCKUP | RESETREAS_SECTAMPER)) != 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn watchdog_reset_is_latched_as_problem() {
        let mut state = DiagnosticsState::new();
        state.note_resetreas(RESETREAS_DOG0);

        assert!(state.unexpected_reset);
        assert!(state.problem_active());
    }

    #[test]
    fn benign_reset_does_not_raise_problem() {
        let mut state = DiagnosticsState::new();
        state.note_resetreas(1);

        assert!(!state.unexpected_reset);
        assert!(!state.problem_active());
    }

    #[test]
    fn pofwarn_latches_problem() {
        let mut state = DiagnosticsState::new();
        state.note_pofwarn();

        assert!(state.pofwarn_seen);
        assert!(state.problem_active());
    }
}
