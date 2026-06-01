//! First-boot setup and recommissioning logic.

use crate::classifier::WindowState;
use crate::gesture::Gesture;

const FACTORY_NEW_DISCOVERY_WINDOW_MS: u32 = 300_000;
const USER_DISCOVERY_WINDOW_MS: u32 = 30_000;
const SHAKE_DISCOVERY_WINDOW_MS: u64 = 4_000;
const TURN_CONFIRM_WINDOW_MS: u64 = 5_000;
const TAP_IDENTIFY_WINDOW_MS: u64 = 1_500;

const SHAKE_DISCOVERY_COUNT: usize = 3;
const TURN_CONFIRM_COUNT: usize = 2;
const TAP_IDENTIFY_COUNT: usize = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), derive(defmt::Format))]
pub enum CalibrationPhase {
    Closed,
    Tilt,
    Open,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), derive(defmt::Format))]
pub enum SetupState {
    FactoryNew,
    Discovery,
    WaitingForClosedCalibration,
    CalibratingClosed,
    WaitingForTiltCalibration,
    CalibratingTilt,
    WaitingForOpenCalibration,
    CalibratingOpen,
    Ready,
    NeedsRecalibration,
    Debug,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), derive(defmt::Format))]
pub enum LedHint {
    Silent,
    DiscoverySlowBlink,
    CalibrationClosedPrompt,
    CalibrationTiltPrompt,
    CalibrationOpenPrompt,
    CalibrationOk,
    IdentifyPulse,
    Attention,
    CalibrationFailed,
    FactoryResetAlternating,
    DebugSolid,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), derive(defmt::Format))]
pub enum MemsInterrupt {
    Wake,
    OrientationChange,
    Tap,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), derive(defmt::Format))]
pub enum MemsGesture {
    StartDiscovery,
    ConfirmMount,
    Identify,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), derive(defmt::Format))]
pub enum SetupEvent {
    Boot { has_calibration: bool },
    Tick,
    MagnetGesture(Gesture),
    MemsGesture(MemsGesture),
    WindowState(WindowState),
    CalibrationCaptured(CalibrationPhase),
    CalibrationRejected(CalibrationPhase),
    CalibrationStored,
    TamperDetected,
    DebugRequested,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SetupDecision {
    pub state: SetupState,
    pub led: LedHint,
    pub discovery_window_ms: Option<u32>,
    pub calibration_phase: Option<CalibrationPhase>,
    pub should_clear_state: bool,
    pub should_enter_debug: bool,
}

impl SetupDecision {
    const fn quiet(state: SetupState) -> Self {
        Self {
            state,
            led: LedHint::Silent,
            discovery_window_ms: None,
            calibration_phase: None,
            should_clear_state: false,
            should_enter_debug: false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SetupController {
    state: SetupState,
    discovery_deadline_ms: Option<u64>,
    last_window_state: WindowState,
}

impl SetupController {
    pub const fn new() -> Self {
        Self {
            state: SetupState::FactoryNew,
            discovery_deadline_ms: None,
            last_window_state: WindowState::Closed,
        }
    }

    pub const fn state(&self) -> SetupState {
        self.state
    }

    pub fn apply(&mut self, event: SetupEvent, at_ms: u64) -> SetupDecision {
        match event {
            SetupEvent::Boot { has_calibration } => {
                if has_calibration {
                    self.state = SetupState::Ready;
                    self.discovery_deadline_ms = None;
                    return SetupDecision::quiet(self.state);
                }

                self.enter_discovery(at_ms, FACTORY_NEW_DISCOVERY_WINDOW_MS)
            }
            SetupEvent::Tick => {
                if self.state == SetupState::Discovery
                    && self
                        .discovery_deadline_ms
                        .is_some_and(|deadline| at_ms >= deadline)
                {
                    self.state = SetupState::FactoryNew;
                    self.discovery_deadline_ms = None;
                }
                SetupDecision::quiet(self.state)
            }
            SetupEvent::MagnetGesture(gesture) => match gesture {
                Gesture::Calibrate => self.request_calibration(),
                Gesture::Pairing => self.enter_discovery(at_ms, USER_DISCOVERY_WINDOW_MS),
                Gesture::FactoryReset => {
                    let mut decision = self.enter_discovery(at_ms, FACTORY_NEW_DISCOVERY_WINDOW_MS);
                    decision.led = LedHint::FactoryResetAlternating;
                    decision.should_clear_state = true;
                    decision
                }
            },
            SetupEvent::MemsGesture(gesture) => match gesture {
                MemsGesture::StartDiscovery => {
                    self.enter_discovery(at_ms, USER_DISCOVERY_WINDOW_MS)
                }
                MemsGesture::ConfirmMount => self.request_calibration(),
                MemsGesture::Identify => {
                    let mut decision = SetupDecision::quiet(self.state);
                    decision.led = LedHint::IdentifyPulse;
                    decision
                }
            },
            SetupEvent::WindowState(state) => {
                self.last_window_state = state;

                if self.state == SetupState::WaitingForClosedCalibration
                    && state == WindowState::Closed
                {
                    self.state = SetupState::CalibratingClosed;
                    let mut decision = SetupDecision::quiet(self.state);
                    decision.led = LedHint::CalibrationClosedPrompt;
                    decision.calibration_phase = Some(CalibrationPhase::Closed);
                    return decision;
                }

                if self.state == SetupState::WaitingForTiltCalibration && state == WindowState::Tilt
                {
                    self.state = SetupState::CalibratingTilt;
                    let mut decision = SetupDecision::quiet(self.state);
                    decision.led = LedHint::CalibrationTiltPrompt;
                    decision.calibration_phase = Some(CalibrationPhase::Tilt);
                    return decision;
                }

                if self.state == SetupState::WaitingForOpenCalibration && state == WindowState::Open
                {
                    self.state = SetupState::CalibratingOpen;
                    let mut decision = SetupDecision::quiet(self.state);
                    decision.led = LedHint::CalibrationOpenPrompt;
                    decision.calibration_phase = Some(CalibrationPhase::Open);
                    return decision;
                }

                SetupDecision::quiet(self.state)
            }
            SetupEvent::CalibrationCaptured(phase) => {
                let mut decision = SetupDecision::quiet(self.state);
                match phase {
                    CalibrationPhase::Closed => {
                        self.state = SetupState::WaitingForTiltCalibration;
                        decision.state = self.state;
                        decision.led = LedHint::CalibrationTiltPrompt;
                    }
                    CalibrationPhase::Tilt => {
                        self.state = SetupState::WaitingForOpenCalibration;
                        decision.state = self.state;
                        decision.led = LedHint::CalibrationOpenPrompt;
                    }
                    CalibrationPhase::Open => {
                        self.state = SetupState::CalibratingOpen;
                        decision.state = self.state;
                    }
                }
                decision
            }
            SetupEvent::CalibrationRejected(phase) => self.retry_calibration(phase),
            SetupEvent::CalibrationStored => {
                self.state = SetupState::Ready;
                self.discovery_deadline_ms = None;
                let mut decision = SetupDecision::quiet(self.state);
                decision.led = LedHint::CalibrationOk;
                decision
            }
            SetupEvent::TamperDetected => {
                self.state = SetupState::NeedsRecalibration;
                let mut decision = SetupDecision::quiet(self.state);
                decision.led = LedHint::Attention;
                decision
            }
            SetupEvent::DebugRequested => {
                self.state = SetupState::Debug;
                let mut decision = SetupDecision::quiet(self.state);
                decision.led = LedHint::DebugSolid;
                decision.should_enter_debug = true;
                decision
            }
        }
    }

    fn enter_discovery(&mut self, at_ms: u64, window_ms: u32) -> SetupDecision {
        self.state = SetupState::Discovery;
        self.discovery_deadline_ms = Some(at_ms + u64::from(window_ms));

        let mut decision = SetupDecision::quiet(self.state);
        decision.led = LedHint::DiscoverySlowBlink;
        decision.discovery_window_ms = Some(window_ms);
        decision
    }

    fn request_calibration(&mut self) -> SetupDecision {
        self.retry_calibration(CalibrationPhase::Closed)
    }

    fn retry_calibration(&mut self, phase: CalibrationPhase) -> SetupDecision {
        if self.last_window_state == Self::phase_window_state(phase) {
            self.state = Self::phase_calibrating_state(phase);
            let mut decision = SetupDecision::quiet(self.state);
            decision.led = Self::phase_led_hint(phase);
            decision.calibration_phase = Some(phase);
            return decision;
        }

        self.state = Self::phase_waiting_state(phase);
        let mut decision = SetupDecision::quiet(self.state);
        decision.led = Self::phase_led_hint(phase);
        decision
    }

    const fn phase_window_state(phase: CalibrationPhase) -> WindowState {
        match phase {
            CalibrationPhase::Closed => WindowState::Closed,
            CalibrationPhase::Tilt => WindowState::Tilt,
            CalibrationPhase::Open => WindowState::Open,
        }
    }

    const fn phase_waiting_state(phase: CalibrationPhase) -> SetupState {
        match phase {
            CalibrationPhase::Closed => SetupState::WaitingForClosedCalibration,
            CalibrationPhase::Tilt => SetupState::WaitingForTiltCalibration,
            CalibrationPhase::Open => SetupState::WaitingForOpenCalibration,
        }
    }

    const fn phase_calibrating_state(phase: CalibrationPhase) -> SetupState {
        match phase {
            CalibrationPhase::Closed => SetupState::CalibratingClosed,
            CalibrationPhase::Tilt => SetupState::CalibratingTilt,
            CalibrationPhase::Open => SetupState::CalibratingOpen,
        }
    }

    const fn phase_led_hint(phase: CalibrationPhase) -> LedHint {
        match phase {
            CalibrationPhase::Closed => LedHint::CalibrationClosedPrompt,
            CalibrationPhase::Tilt => LedHint::CalibrationTiltPrompt,
            CalibrationPhase::Open => LedHint::CalibrationOpenPrompt,
        }
    }
}

impl Default for SetupController {
    fn default() -> Self {
        Self::new()
    }
}

impl SetupState {
    pub const fn ble_discovery_active(self) -> bool {
        matches!(self, SetupState::Discovery)
    }

    pub const fn calibration_required(self) -> bool {
        matches!(
            self,
            SetupState::WaitingForClosedCalibration
                | SetupState::CalibratingClosed
                | SetupState::WaitingForTiltCalibration
                | SetupState::CalibratingTilt
                | SetupState::WaitingForOpenCalibration
                | SetupState::CalibratingOpen
                | SetupState::NeedsRecalibration
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MemsGestureDetector {
    shakes_ms: [u64; SHAKE_DISCOVERY_COUNT],
    shake_len: usize,
    turns_ms: [u64; TURN_CONFIRM_COUNT],
    turn_len: usize,
    taps_ms: [u64; TAP_IDENTIFY_COUNT],
    tap_len: usize,
}

impl MemsGestureDetector {
    pub const fn new() -> Self {
        Self {
            shakes_ms: [0; SHAKE_DISCOVERY_COUNT],
            shake_len: 0,
            turns_ms: [0; TURN_CONFIRM_COUNT],
            turn_len: 0,
            taps_ms: [0; TAP_IDENTIFY_COUNT],
            tap_len: 0,
        }
    }

    pub fn record(&mut self, interrupt: MemsInterrupt, at_ms: u64) -> Option<MemsGesture> {
        match interrupt {
            MemsInterrupt::Wake => {
                push_recent(
                    &mut self.shakes_ms,
                    &mut self.shake_len,
                    at_ms,
                    SHAKE_DISCOVERY_WINDOW_MS,
                );
                if self.shake_len >= SHAKE_DISCOVERY_COUNT {
                    self.shake_len = 0;
                    return Some(MemsGesture::StartDiscovery);
                }
            }
            MemsInterrupt::OrientationChange => {
                push_recent(
                    &mut self.turns_ms,
                    &mut self.turn_len,
                    at_ms,
                    TURN_CONFIRM_WINDOW_MS,
                );
                if self.turn_len >= TURN_CONFIRM_COUNT {
                    self.turn_len = 0;
                    return Some(MemsGesture::ConfirmMount);
                }
            }
            MemsInterrupt::Tap => {
                push_recent(
                    &mut self.taps_ms,
                    &mut self.tap_len,
                    at_ms,
                    TAP_IDENTIFY_WINDOW_MS,
                );
                if self.tap_len >= TAP_IDENTIFY_COUNT {
                    self.tap_len = 0;
                    return Some(MemsGesture::Identify);
                }
            }
        }

        None
    }
}

impl Default for MemsGestureDetector {
    fn default() -> Self {
        Self::new()
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn factory_new_boot_starts_discovery_window() {
        let mut setup = SetupController::new();

        let decision = setup.apply(
            SetupEvent::Boot {
                has_calibration: false,
            },
            1_000,
        );

        assert_eq!(setup.state(), SetupState::Discovery);
        assert_eq!(
            decision.discovery_window_ms,
            Some(FACTORY_NEW_DISCOVERY_WINDOW_MS)
        );
        assert_eq!(decision.led, LedHint::DiscoverySlowBlink);
    }

    #[test]
    fn calibrated_boot_enters_ready_silently() {
        let mut setup = SetupController::new();

        let decision = setup.apply(
            SetupEvent::Boot {
                has_calibration: true,
            },
            1_000,
        );

        assert_eq!(setup.state(), SetupState::Ready);
        assert_eq!(decision, SetupDecision::quiet(SetupState::Ready));
    }

    #[test]
    fn discovery_times_out_back_to_factory_new() {
        let mut setup = SetupController::new();
        let _ = setup.apply(
            SetupEvent::Boot {
                has_calibration: false,
            },
            0,
        );

        let decision = setup.apply(
            SetupEvent::Tick,
            u64::from(FACTORY_NEW_DISCOVERY_WINDOW_MS) + 1,
        );

        assert_eq!(setup.state(), SetupState::FactoryNew);
        assert_eq!(decision.state, SetupState::FactoryNew);
    }

    #[test]
    fn pairing_gesture_reopens_short_discovery_window() {
        let mut setup = SetupController::new();
        let _ = setup.apply(
            SetupEvent::Boot {
                has_calibration: true,
            },
            0,
        );

        let decision = setup.apply(SetupEvent::MagnetGesture(Gesture::Pairing), 5_000);

        assert_eq!(setup.state(), SetupState::Discovery);
        assert_eq!(decision.discovery_window_ms, Some(USER_DISCOVERY_WINDOW_MS));
    }

    #[test]
    fn mount_confirm_waits_for_closed_then_starts_calibration() {
        let mut setup = SetupController::new();
        let _ = setup.apply(
            SetupEvent::Boot {
                has_calibration: false,
            },
            0,
        );
        let _ = setup.apply(SetupEvent::WindowState(WindowState::Open), 100);

        let waiting = setup.apply(SetupEvent::MemsGesture(MemsGesture::ConfirmMount), 200);
        assert_eq!(setup.state(), SetupState::WaitingForClosedCalibration);
        assert_eq!(waiting.calibration_phase, None);

        let capture = setup.apply(SetupEvent::WindowState(WindowState::Closed), 300);
        assert_eq!(setup.state(), SetupState::CalibratingClosed);
        assert_eq!(capture.calibration_phase, Some(CalibrationPhase::Closed));
        assert_eq!(capture.led, LedHint::CalibrationClosedPrompt);
    }

    #[test]
    fn calibration_completion_enters_ready() {
        let mut setup = SetupController::new();
        let _ = setup.apply(
            SetupEvent::Boot {
                has_calibration: false,
            },
            0,
        );
        let _ = setup.apply(SetupEvent::WindowState(WindowState::Closed), 100);
        let _ = setup.apply(SetupEvent::MagnetGesture(Gesture::Calibrate), 200);
        let tilt = setup.apply(
            SetupEvent::CalibrationCaptured(CalibrationPhase::Closed),
            250,
        );
        assert_eq!(setup.state(), SetupState::WaitingForTiltCalibration);
        assert_eq!(tilt.led, LedHint::CalibrationTiltPrompt);

        let tilt_capture = setup.apply(SetupEvent::WindowState(WindowState::Tilt), 300);
        assert_eq!(setup.state(), SetupState::CalibratingTilt);
        assert_eq!(tilt_capture.calibration_phase, Some(CalibrationPhase::Tilt));

        let open = setup.apply(SetupEvent::CalibrationCaptured(CalibrationPhase::Tilt), 350);
        assert_eq!(setup.state(), SetupState::WaitingForOpenCalibration);
        assert_eq!(open.led, LedHint::CalibrationOpenPrompt);

        let open_capture = setup.apply(SetupEvent::WindowState(WindowState::Open), 400);
        assert_eq!(setup.state(), SetupState::CalibratingOpen);
        assert_eq!(open_capture.calibration_phase, Some(CalibrationPhase::Open));

        let decision = setup.apply(SetupEvent::CalibrationStored, 450);

        assert_eq!(setup.state(), SetupState::Ready);
        assert_eq!(decision.led, LedHint::CalibrationOk);
    }

    #[test]
    fn calibration_rejection_retries_current_phase_when_window_state_matches() {
        let mut setup = SetupController::new();
        let _ = setup.apply(
            SetupEvent::Boot {
                has_calibration: false,
            },
            0,
        );
        let _ = setup.apply(SetupEvent::WindowState(WindowState::Closed), 100);

        let decision = setup.apply(
            SetupEvent::CalibrationRejected(CalibrationPhase::Closed),
            150,
        );

        assert_eq!(setup.state(), SetupState::CalibratingClosed);
        assert_eq!(decision.led, LedHint::CalibrationClosedPrompt);
        assert_eq!(decision.calibration_phase, Some(CalibrationPhase::Closed));
    }

    #[test]
    fn calibration_rejection_waits_for_phase_when_window_state_differs() {
        let mut setup = SetupController::new();
        let _ = setup.apply(
            SetupEvent::Boot {
                has_calibration: false,
            },
            0,
        );
        let _ = setup.apply(SetupEvent::WindowState(WindowState::Closed), 100);
        let _ = setup.apply(SetupEvent::MagnetGesture(Gesture::Calibrate), 200);
        let _ = setup.apply(
            SetupEvent::CalibrationCaptured(CalibrationPhase::Closed),
            250,
        );
        let _ = setup.apply(SetupEvent::WindowState(WindowState::Open), 300);

        let decision = setup.apply(SetupEvent::CalibrationRejected(CalibrationPhase::Tilt), 350);

        assert_eq!(setup.state(), SetupState::WaitingForTiltCalibration);
        assert_eq!(decision.led, LedHint::CalibrationTiltPrompt);
        assert_eq!(decision.calibration_phase, None);
    }

    #[test]
    fn reoriented_tamper_requires_recalibration() {
        let mut setup = SetupController::new();
        let _ = setup.apply(
            SetupEvent::Boot {
                has_calibration: true,
            },
            0,
        );

        let decision = setup.apply(SetupEvent::TamperDetected, 1_000);

        assert_eq!(setup.state(), SetupState::NeedsRecalibration);
        assert_eq!(decision.led, LedHint::Attention);
    }

    #[test]
    fn factory_reset_clears_state_and_restarts_discovery() {
        let mut setup = SetupController::new();
        let _ = setup.apply(
            SetupEvent::Boot {
                has_calibration: true,
            },
            0,
        );

        let decision = setup.apply(SetupEvent::MagnetGesture(Gesture::FactoryReset), 2_000);

        assert_eq!(setup.state(), SetupState::Discovery);
        assert!(decision.should_clear_state);
        assert_eq!(decision.led, LedHint::FactoryResetAlternating);
        assert_eq!(
            decision.discovery_window_ms,
            Some(FACTORY_NEW_DISCOVERY_WINDOW_MS)
        );
    }

    #[test]
    fn debug_request_enters_debug_mode() {
        let mut setup = SetupController::new();

        let decision = setup.apply(SetupEvent::DebugRequested, 10_000);

        assert_eq!(setup.state(), SetupState::Debug);
        assert!(decision.should_enter_debug);
        assert_eq!(decision.led, LedHint::DebugSolid);
    }

    #[test]
    fn discovery_state_requests_connectable_ble() {
        assert!(SetupState::Discovery.ble_discovery_active());
        assert!(!SetupState::Ready.ble_discovery_active());
    }

    #[test]
    fn calibration_related_states_require_follow_up() {
        assert!(SetupState::WaitingForClosedCalibration.calibration_required());
        assert!(SetupState::CalibratingClosed.calibration_required());
        assert!(SetupState::WaitingForTiltCalibration.calibration_required());
        assert!(SetupState::CalibratingTilt.calibration_required());
        assert!(SetupState::WaitingForOpenCalibration.calibration_required());
        assert!(SetupState::CalibratingOpen.calibration_required());
        assert!(SetupState::NeedsRecalibration.calibration_required());
        assert!(!SetupState::Ready.calibration_required());
    }

    #[test]
    fn shake_pattern_starts_discovery() {
        let mut detector = MemsGestureDetector::new();

        assert_eq!(detector.record(MemsInterrupt::Wake, 0), None);
        assert_eq!(detector.record(MemsInterrupt::Wake, 1_000), None);
        assert_eq!(
            detector.record(MemsInterrupt::Wake, 2_500),
            Some(MemsGesture::StartDiscovery)
        );
    }

    #[test]
    fn orientation_pattern_confirms_mount() {
        let mut detector = MemsGestureDetector::new();

        assert_eq!(detector.record(MemsInterrupt::OrientationChange, 0), None);
        assert_eq!(
            detector.record(MemsInterrupt::OrientationChange, 2_000),
            Some(MemsGesture::ConfirmMount)
        );
    }

    #[test]
    fn double_tap_identifies_device() {
        let mut detector = MemsGestureDetector::new();

        assert_eq!(detector.record(MemsInterrupt::Tap, 0), None);
        assert_eq!(
            detector.record(MemsInterrupt::Tap, 800),
            Some(MemsGesture::Identify)
        );
    }

    #[test]
    fn slow_motion_patterns_do_not_trigger() {
        let mut detector = MemsGestureDetector::new();

        assert_eq!(detector.record(MemsInterrupt::Wake, 0), None);
        assert_eq!(detector.record(MemsInterrupt::Wake, 5_000), None);
        assert_eq!(detector.record(MemsInterrupt::Wake, 10_000), None);
        assert_eq!(detector.record(MemsInterrupt::Tap, 20_000), None);
        assert_eq!(detector.record(MemsInterrupt::Tap, 22_000), None);
    }
}
