use defmt::*;
use embassy_futures::select::{Either, select};
use embassy_time::Instant;
use embassy_time::{Duration, Timer};

use crate::{SETUP_CHANGED, SETUP_EVENT, SETUP_STATE, WINDOW_CALIBRATION_REQUEST};
use window_sensor::setup::{LedHint, SetupController, SetupDecision, SetupEvent, SetupState};

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
        "[SETUP] state={} led={} discovery={} capture={} clear={} debug={}",
        state_str(decision.state),
        led_str(decision.led),
        decision.discovery_window_ms.unwrap_or(0),
        decision.should_capture_calibration,
        decision.should_clear_state,
        decision.should_enter_debug,
    );
    SETUP_STATE.lock(|state| *state.borrow_mut() = decision.state);
    if decision.should_capture_calibration {
        WINDOW_CALIBRATION_REQUEST.signal(());
    }
    SETUP_CHANGED.signal(decision);
}

fn state_str(state: SetupState) -> &'static str {
    match state {
        SetupState::FactoryNew => "FACTORY_NEW",
        SetupState::Discovery => "DISCOVERY",
        SetupState::WaitingForClosedCalibration => "WAIT_CLOSED_CAL",
        SetupState::Calibrating => "CALIBRATING",
        SetupState::Ready => "READY",
        SetupState::NeedsRecalibration => "NEEDS_RECAL",
        SetupState::Debug => "DEBUG",
    }
}

fn led_str(led: LedHint) -> &'static str {
    match led {
        LedHint::Silent => "SILENT",
        LedHint::DiscoverySlowBlink => "DISCOVERY_BLINK",
        LedHint::CalibrationConfirm => "CAL_CONFIRM",
        LedHint::CalibrationOk => "CAL_OK",
        LedHint::IdentifyPulse => "IDENTIFY",
        LedHint::Attention => "ATTENTION",
        LedHint::FactoryResetAlternating => "RESET",
        LedHint::DebugSolid => "DEBUG_SOLID",
    }
}
