use defmt::*;
use embassy_futures::select::{Either, select};
use embassy_time::Timer;
use embassy_time::Instant;
use embassy_time::Duration;

use crate::drivers::led::Led;
use crate::{SETUP_CHANGED, SETUP_EVENT, SETUP_LED_HINT, SETUP_STATE, WINDOW_CALIBRATION_REQUEST};
use window_sensor::setup::{CalibrationPhase, LedHint, SetupController, SetupDecision, SetupEvent, SetupState};

const SETUP_TICK_PERIOD: Duration = Duration::from_secs(1);

#[embassy_executor::task]
pub async fn setup_task(has_calibration: bool) {
    info!("[SETUP] Task started");

    let mut controller = SetupController::new();
    let boot = controller.apply(SetupEvent::Boot { has_calibration }, 0);
    publish(boot);

    loop {
        let event = match select(SETUP_EVENT.wait(), Timer::after(SETUP_TICK_PERIOD)).await {
            Either::First(event) => event,
            Either::Second(_) => SetupEvent::Tick,
        };
        let decision = controller.apply(event, Instant::now().as_millis());
        publish(decision);
    }
}

fn publish(decision: SetupDecision) {
    info!(
        "[SETUP] state={} led={} discovery={} phase={} clear={} debug={}",
        state_str(decision.state),
        led_str(decision.led),
        decision.discovery_window_ms.unwrap_or(0),
        phase_str(decision.calibration_phase),
        decision.should_clear_state,
        decision.should_enter_debug,
    );
    SETUP_STATE.lock(|state| *state.borrow_mut() = decision.state);
    SETUP_LED_HINT.signal(decision.led);
    if let Some(phase) = decision.calibration_phase {
        WINDOW_CALIBRATION_REQUEST.signal(phase);
    }
    SETUP_CHANGED.signal(decision);
}

#[embassy_executor::task]
pub async fn setup_led_task(mut led_blue: Led<'static>) {
    let mut current = LedHint::Silent;

    loop {
        current = match current {
            LedHint::Silent
            | LedHint::DiscoverySlowBlink
            | LedHint::IdentifyPulse
            | LedHint::Attention
            | LedHint::FactoryResetAlternating
            | LedHint::DebugSolid
            | LedHint::CalibrationFailed => SETUP_LED_HINT.wait().await,
            _ => match select(SETUP_LED_HINT.wait(), play_blue_pattern(&mut led_blue, current)).await {
                Either::First(next) => next,
                Either::Second(_) => current,
            },
        };
    }
}

fn state_str(state: SetupState) -> &'static str {
    match state {
        SetupState::FactoryNew => "FACTORY_NEW",
        SetupState::Discovery => "DISCOVERY",
        SetupState::WaitingForClosedCalibration => "WAIT_CLOSED_CAL",
        SetupState::CalibratingClosed => "CAL_CLOSED",
        SetupState::WaitingForTiltCalibration => "WAIT_TILT_CAL",
        SetupState::CalibratingTilt => "CAL_TILT",
        SetupState::WaitingForOpenCalibration => "WAIT_OPEN_CAL",
        SetupState::CalibratingOpen => "CAL_OPEN",
        SetupState::Ready => "READY",
        SetupState::NeedsRecalibration => "NEEDS_RECAL",
        SetupState::Debug => "DEBUG",
    }
}

fn led_str(led: LedHint) -> &'static str {
    match led {
        LedHint::Silent => "SILENT",
        LedHint::DiscoverySlowBlink => "DISCOVERY_BLINK",
        LedHint::CalibrationClosedPrompt => "CAL_CLOSED",
        LedHint::CalibrationTiltPrompt => "CAL_TILT",
        LedHint::CalibrationOpenPrompt => "CAL_OPEN",
        LedHint::CalibrationOk => "CAL_OK",
        LedHint::IdentifyPulse => "IDENTIFY",
        LedHint::Attention => "ATTENTION",
        LedHint::CalibrationFailed => "CAL_FAILED",
        LedHint::FactoryResetAlternating => "RESET",
        LedHint::DebugSolid => "DEBUG_SOLID",
    }
}

fn phase_str(phase: Option<CalibrationPhase>) -> &'static str {
    match phase {
        Some(CalibrationPhase::Closed) => "CLOSED",
        Some(CalibrationPhase::Tilt) => "TILT",
        Some(CalibrationPhase::Open) => "OPEN",
        None => "-",
    }
}

async fn play_blue_pattern(led_blue: &mut Led<'static>, hint: LedHint) {
    match hint {
        LedHint::CalibrationClosedPrompt => led_blue.blink(80, 220, 1).await,
        LedHint::CalibrationTiltPrompt => led_blue.blink(80, 180, 2).await,
        LedHint::CalibrationOpenPrompt => led_blue.blink(80, 180, 3).await,
        LedHint::CalibrationOk => {
            led_blue.blink(120, 120, 2).await;
            Timer::after_millis(400).await;
        }
        _ => Timer::after_millis(200).await,
    }
}
