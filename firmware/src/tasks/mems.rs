use defmt::*;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_futures::select::{Either, select};
use embassy_nrf::gpio::Input;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Instant, Timer};
use lis2dtw12::interface::{I2CInterface as LisI2cInterface, SlaveAddr as LisSlaveAddr};
use lis2dtw12::{
    BandwidthSelection, FullScale, Int1PadConfig, Int2PadConfig, Lis2dtw12, Mode, OutputDataRate,
    TapPriority, Threshold6D,
};

use crate::{BUTTON_EVENT, SETUP_EVENT, SETUP_STATE, SharedI2cBus};
use window_sensor::mems::{
    IdsDetector, IdsSignal, MemsConfig, MemsInterruptConfig, MemsOrientationThreshold,
    ids_enabled_for_setup_state, setup_gestures_enabled_for_state,
};
use window_sensor::mems_button::{MemsButtonDetector, MemsButtonSignal};
use window_sensor::setup::{MemsGestureDetector, MemsInterrupt, SetupEvent, SetupState};

type LisBus = I2cDevice<'static, NoopRawMutex, embassy_nrf::twim::Twim<'static>>;
type LisDevice = Lis2dtw12<LisI2cInterface<LisBus>>;

const RETRY_DELAY: Duration = Duration::from_secs(5);

#[embassy_executor::task]
pub async fn mems_task(
    bus: &'static SharedI2cBus,
    mut mems_int1: Input<'static>,
    mut mems_int2: Input<'static>,
    config: MemsConfig,
) {
    info!("[MEMS] Task started");

    let mut button_detector = MemsButtonDetector::new(config.button);
    let mut gesture_detector = MemsGestureDetector::new();
    let mut ids_detector = IdsDetector::new(config.ids);

    loop {
        let mut lis = match init_lis(bus, config.interrupts) {
            Ok(lis) => {
                info!("[MEMS] LIS2DTW12 configured for interrupts");
                lis
            }
            Err(_) => {
                error!("[MEMS] Failed to initialize LIS2DTW12 interrupts");
                Timer::after(RETRY_DELAY).await;
                continue;
            }
        };

        loop {
            let now_ms = Instant::now().as_millis();
            match next_button_deadline_ms(&button_detector, now_ms) {
                Some(deadline_ms) if deadline_ms <= now_ms => {
                    flush_button_tick(&mut button_detector, now_ms);
                    continue;
                }
                Some(deadline_ms) => {
                    let wait_ms = deadline_ms.saturating_sub(now_ms);
                    match select(
                        select(mems_int1.wait_for_low(), mems_int2.wait_for_low()),
                        Timer::after_millis(wait_ms),
                    )
                    .await
                    {
                        Either::First(Either::First(_)) => {
                            if handle_interrupt(
                                &mut lis,
                                MemsLine::Int1,
                                &mut gesture_detector,
                                &mut ids_detector,
                                &mut button_detector,
                            )
                            .is_err()
                            {
                                error!("[MEMS] INT1 handling failed");
                                break;
                            }
                        }
                        Either::First(Either::Second(_)) => {
                            if handle_interrupt(
                                &mut lis,
                                MemsLine::Int2,
                                &mut gesture_detector,
                                &mut ids_detector,
                                &mut button_detector,
                            )
                            .is_err()
                            {
                                error!("[MEMS] INT2 handling failed");
                                break;
                            }
                        }
                        Either::Second(_) => flush_button_tick(&mut button_detector, deadline_ms),
                    }
                }
                None => match select(mems_int1.wait_for_low(), mems_int2.wait_for_low()).await {
                    Either::First(_) => {
                        if handle_interrupt(
                            &mut lis,
                            MemsLine::Int1,
                            &mut gesture_detector,
                            &mut ids_detector,
                            &mut button_detector,
                        )
                        .is_err()
                        {
                            error!("[MEMS] INT1 handling failed");
                            break;
                        }
                    }
                    Either::Second(_) => {
                        if handle_interrupt(
                            &mut lis,
                            MemsLine::Int2,
                            &mut gesture_detector,
                            &mut ids_detector,
                            &mut button_detector,
                        )
                        .is_err()
                        {
                            error!("[MEMS] INT2 handling failed");
                            break;
                        }
                    }
                },
            }
        }

        Timer::after(RETRY_DELAY).await;
    }
}

#[derive(Clone, Copy)]
enum MemsLine {
    Int1,
    Int2,
}

fn init_lis(bus: &'static SharedI2cBus, config: MemsInterruptConfig) -> Result<LisDevice, ()> {
    let lis_bus = LisI2cInterface::new(I2cDevice::new(bus), LisSlaveAddr::Alternative(true));
    let mut lis = Lis2dtw12::new(lis_bus);

    if lis.get_device_id().map_err(|_| ())? != 0x44 {
        return Err(());
    }

    lis.reset_settings_blocking().map_err(|_| ())?;
    lis.set_mode(Mode::ContinuousLowPower1).map_err(|_| ())?;
    lis.set_output_data_rate(OutputDataRate::Hz25)
        .map_err(|_| ())?;
    lis.set_full_scale(FullScale::G2).map_err(|_| ())?;
    lis.set_bandwidth(BandwidthSelection::OdrDiv4)
        .map_err(|_| ())?;
    lis.enable_continuous_update(true).map_err(|_| ())?;

    // Keep the MEMS path low-power while still responsive enough for taps and motion.
    lis.set_wake_up_threshold(config.wake_up_threshold)
        .map_err(|_| ())?;
    lis.set_wake_up_duration(config.wake_up_duration)
        .map_err(|_| ())?;
    lis.enable_sleep_mode(config.sleep_mode_enabled)
        .map_err(|_| ())?;

    lis.enable_4d_detection(config.orientation_detection_enabled)
        .map_err(|_| ())?;
    lis.set_6d_threshold(map_orientation_threshold(config.orientation_threshold))
        .map_err(|_| ())?;

    lis.set_tap_priority(TapPriority::XYZ).map_err(|_| ())?;
    lis.enable_xyz_tap_detection(
        config.tap_detection_enabled,
        config.tap_detection_enabled,
        config.tap_detection_enabled,
    )
        .map_err(|_| ())?;
    lis.set_x_tap_threshold(config.tap_threshold_x)
        .map_err(|_| ())?;
    lis.set_y_tap_threshold(config.tap_threshold_y)
        .map_err(|_| ())?;
    lis.set_z_tap_threshold(config.tap_threshold_z)
        .map_err(|_| ())?;
    lis.set_tap_quiet_time(config.tap_quiet_time)
        .map_err(|_| ())?;
    lis.set_tap_shock_time(config.tap_shock_time)
        .map_err(|_| ())?;
    lis.set_double_tap_latency(config.double_tap_latency)
        .map_err(|_| ())?;
    lis.enable_double_tap_detection(config.tap_detection_enabled)
        .map_err(|_| ())?;

    lis.configure_int1_pad(Int1PadConfig {
        int1_6d: config.orientation_detection_enabled,
        int1_single_tap: config.tap_detection_enabled,
        int1_wu: true,
        int1_ff: false,
        int1_tap: config.tap_detection_enabled,
        int1_diff5: false,
        int1_fth: false,
        int1_drdy: false,
    })
    .map_err(|_| ())?;

    lis.configure_int2_pad(Int2PadConfig {
        int2_sleep_state: false,
        int2_sleep_chg: true,
        int2_boot: false,
        int2_drdy_t: false,
        int2_ovr: false,
        int2_diff5: false,
        int2_fth: false,
        int2_drdy: false,
    })
    .map_err(|_| ())?;

    lis.enable_interrupts(true).map_err(|_| ())?;

    Ok(lis)
}

fn map_orientation_threshold(value: MemsOrientationThreshold) -> Threshold6D {
    match value {
        MemsOrientationThreshold::Deg50 => Threshold6D::Deg50,
        MemsOrientationThreshold::Deg60 => Threshold6D::Deg60,
        MemsOrientationThreshold::Deg70 => Threshold6D::Deg70,
        MemsOrientationThreshold::Deg80 => Threshold6D::Deg80,
    }
}

fn handle_interrupt(
    lis: &mut LisDevice,
    line: MemsLine,
    gesture_detector: &mut MemsGestureDetector,
    ids_detector: &mut IdsDetector,
    button_detector: &mut MemsButtonDetector,
) -> Result<(), ()> {
    let at_ms = Instant::now().as_millis();
    let setup_state = current_setup_state();
    let ids_enabled = ids_enabled_for_setup_state(setup_state);
    let setup_gestures_enabled = setup_gestures_enabled_for_state(setup_state);
    let sources = lis.get_all_sources().map_err(|_| ())?;

    trace!(
        "[MEMS] {:?} wu={} tap={} dtap={} 6d={} sleepchg={}",
        line_str(line),
        sources.all_interrupt_sources.wake_up_interrupt,
        sources.all_interrupt_sources.single_tap_interrupt,
        sources.all_interrupt_sources.double_tap_interrupt,
        sources.all_interrupt_sources.six_d_interrupt,
        sources.all_interrupt_sources.sleep_change_interrupt,
    );

    if sources.all_interrupt_sources.wake_up_interrupt || sources.wake_up_source.wake_up_event {
        if ids_enabled {
            signal_tamper_if_triggered(ids_detector.record(IdsSignal::Wake, at_ms));
        }
        if setup_gestures_enabled
            && let Some(gesture) = gesture_detector.record(MemsInterrupt::Wake, at_ms)
        {
            SETUP_EVENT.signal(SetupEvent::MemsGesture(gesture));
        }
        let _ = button_detector.record(MemsButtonSignal::MotionStart, at_ms);
    }

    if sources.all_interrupt_sources.single_tap_interrupt || sources.tap_source.single_tap_event {
        handle_tap(
            setup_gestures_enabled,
            ids_enabled,
            gesture_detector,
            ids_detector,
            button_detector,
            at_ms,
            1,
        );
    }

    if sources.all_interrupt_sources.double_tap_interrupt || sources.tap_source.double_tap_event {
        handle_tap(
            setup_gestures_enabled,
            ids_enabled,
            gesture_detector,
            ids_detector,
            button_detector,
            at_ms,
            2,
        );
    }

    if sources.all_interrupt_sources.six_d_interrupt || sources.six_d_source.position_change_event {
        if ids_enabled {
            signal_tamper_if_triggered(ids_detector.record(IdsSignal::OrientationChange, at_ms));
        }
        if setup_gestures_enabled
            && let Some(gesture) = gesture_detector.record(MemsInterrupt::OrientationChange, at_ms)
        {
            SETUP_EVENT.signal(SetupEvent::MemsGesture(gesture));
        }
    }

    if (sources.all_interrupt_sources.sleep_change_interrupt || sources.wake_up_source.sleep_event)
        && let Some(button) = button_detector.record(MemsButtonSignal::MotionEnd, at_ms)
    {
        BUTTON_EVENT.signal(button);
    }

    Ok(())
}

fn handle_tap(
    setup_gestures_enabled: bool,
    ids_enabled: bool,
    gesture_detector: &mut MemsGestureDetector,
    ids_detector: &mut IdsDetector,
    button_detector: &mut MemsButtonDetector,
    at_ms: u64,
    count: usize,
) {
    for _ in 0..count {
        if setup_gestures_enabled
            && let Some(gesture) = gesture_detector.record(MemsInterrupt::Tap, at_ms)
        {
            SETUP_EVENT.signal(SetupEvent::MemsGesture(gesture));
        }
        if ids_enabled {
            signal_tamper_if_triggered(ids_detector.record(IdsSignal::Tap, at_ms));
        }
        let _ = button_detector.record(MemsButtonSignal::Knock, at_ms);
    }
}

fn signal_tamper_if_triggered(trigger: Option<window_sensor::mems::IdsTrigger>) {
    if trigger.is_some() {
        SETUP_EVENT.signal(SetupEvent::TamperDetected);
    }
}

fn flush_button_tick(button_detector: &mut MemsButtonDetector, at_ms: u64) {
    if let Some(button) = button_detector.record(MemsButtonSignal::Tick, at_ms) {
        BUTTON_EVENT.signal(button);
    }
}

fn current_setup_state() -> SetupState {
    SETUP_STATE.lock(|state| *state.borrow())
}

fn next_button_deadline_ms(button_detector: &MemsButtonDetector, now_ms: u64) -> Option<u64> {
    button_detector.next_deadline_ms(now_ms)
}

fn line_str(line: MemsLine) -> &'static str {
    match line {
        MemsLine::Int1 => "INT1",
        MemsLine::Int2 => "INT2",
    }
}
