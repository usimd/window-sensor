//! Window state machine task — INT-driven wake, burst capture, classify, signal BLE.
//!
//! Flow:
//! 1. TMAG5273 in wake-and-sleep mode (~1 µA) with threshold INT
//! 2. When magnetic field crosses threshold → INT fires → GPIOTE wakes MCU
//! 3. Switch TMAG to continuous/burst mode (20 Hz)
//! 4. Capture 10–12 samples (0.5–0.6 s window)
//! 5. Run classifier (collapse → TILT, gradual → OPEN)
//! 6. Return to wake-and-sleep mode
//! 7. Signal BLE task with new state

use defmt::*;
use embassy_futures::select::{Either, select};
use embassy_nrf::gpio::Input;
use embassy_time::{Instant, Timer};

use crate::SharedI2cBus;
use crate::{SETUP_EVENT, STATE_CHANGED, SettingsFlash, WINDOW_CALIBRATION_REQUEST};
use window_sensor::classifier::{self, WindowState};
use window_sensor::gesture::{Gesture, GestureDetector};
use window_sensor::setup::{CalibrationPhase, SetupEvent};
use window_sensor::settings::{SocSettings, load_soc_settings, save_soc_settings};
use window_sensor::window_tuning::{self, WindowCalibration};

use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_time::Delay;
use tmag5273::{
    ConfigBuilder as TmagConfigBuilder, ConversionAverage, DeviceVariant, InterruptConfig,
    InterruptMode, InterruptState, MagneticChannel, MagneticReading, MagneticThresholdDirection,
    OperatingMode, Range, SleepTime, ThresholdConfig, ThresholdCrossingCount, ThresholdHysteresis,
    Tmag5273,
};

/// Window event sent to BLE task
#[derive(Clone, Copy)]
pub struct WindowEvent {
    pub state: WindowState,
    /// Peak magnitude observed during the transition (mT × 10 as u16 for compactness)
    pub peak_mt_x10: u16,
}

/// Window task entry point.
///
/// Owns the I2C bus (exclusive access — single-task-per-bus model).
/// `hall_int` is the TMAG5273 INT pin (active-low, pulled high externally).
#[embassy_executor::task]
pub async fn window_task(
    bus: &'static SharedI2cBus,
    mut hall_int: Input<'static>,
    settings_flash: &'static SettingsFlash,
) {
    info!("[WINDOW] Task started");

    let tmag_bus = I2cDevice::new(bus);
    let mut tmag = match init_tmag(tmag_bus) {
        Ok(sensor) => sensor,
        Err(_) => {
            error!("[WINDOW] Failed to initialize TMAG5273");
            loop {
                Timer::after_secs(5).await;
            }
        }
    };
    let mut current_state = WindowState::Closed;
    let mut gestures = GestureDetector::new();
    let mut stored = load_settings(settings_flash);
    let mut calibration = stored.window_calibration();
    let mut pending_calibration = calibration;
    let mut wake_threshold_mt = window_tuning::wake_threshold_mt(calibration);
    let mut closed_threshold_mt = window_tuning::closed_threshold_mt(calibration);

    // Initial configuration: wake-and-sleep with threshold
    match configure_tmag_wake_and_sleep(&mut tmag, wake_threshold_mt) {
        Ok(()) => info!(
            "[WINDOW] TMAG5273 configured: wake-and-sleep, thr={}mT",
            wake_threshold_mt as u32
        ),
        Err(_) => {
            error!("[WINDOW] Failed to configure TMAG5273!");
            // Retry loop
            loop {
                Timer::after_secs(5).await;
                if configure_tmag_wake_and_sleep(&mut tmag, wake_threshold_mt).is_ok() {
                    info!("[WINDOW] TMAG5273 recovered");
                    break;
                }
            }
        }
    }

    loop {
        let requested_calibration = match select(
            WINDOW_CALIBRATION_REQUEST.wait(),
            hall_int.wait_for_low(),
        )
        .await
        {
            Either::First(phase) => Some(phase),
            Either::Second(_) => None,
        };

        if let Some(phase) = requested_calibration {
            match capture_baseline_mt(&mut tmag).await {
                Ok(Some(baseline_mt)) => {
                    match phase {
                        CalibrationPhase::Closed => {
                            if window_tuning::is_valid_closed_baseline_mt(baseline_mt) {
                                let mut next = pending_calibration.unwrap_or(WindowCalibration {
                                    closed_mt: baseline_mt,
                                    tilt_mt: baseline_mt / 2.0,
                                    open_mt: 0.0,
                                });
                                next.closed_mt = baseline_mt;
                                pending_calibration = Some(next);
                                info!("[WINDOW] Captured CLOSED baseline={}mT", baseline_mt as u32);
                                SETUP_EVENT.signal(SetupEvent::CalibrationCaptured(phase));
                            } else {
                                warn!("[WINDOW] Rejected CLOSED baseline={}mT", baseline_mt as u32);
                                SETUP_EVENT.signal(SetupEvent::CalibrationRejected(phase));
                            }
                        }
                        CalibrationPhase::Tilt => {
                            if let Some(mut next) = pending_calibration {
                                next.tilt_mt = baseline_mt;
                                pending_calibration = Some(next);
                                info!("[WINDOW] Captured TILT baseline={}mT", baseline_mt as u32);
                                SETUP_EVENT.signal(SetupEvent::CalibrationCaptured(phase));
                            } else {
                                warn!("[WINDOW] Ignored TILT capture without CLOSED baseline");
                                SETUP_EVENT.signal(SetupEvent::CalibrationRejected(phase));
                            }
                        }
                        CalibrationPhase::Open => {
                            if let Some(mut next) = pending_calibration {
                                next.open_mt = baseline_mt;
                                if window_tuning::is_valid_window_calibration(next) {
                                    stored.window_calibrated = true;
                                    stored.window_closed_baseline_mt_x10 = (next.closed_mt * 10.0) as u16;
                                    stored.window_tilt_baseline_mt_x10 = (next.tilt_mt * 10.0) as u16;
                                    stored.window_open_baseline_mt_x10 = (next.open_mt * 10.0) as u16;
                                    persist_settings(settings_flash, stored);
                                    calibration = Some(next);
                                    pending_calibration = calibration;
                                    wake_threshold_mt = window_tuning::wake_threshold_mt(calibration);
                                    closed_threshold_mt = window_tuning::closed_threshold_mt(calibration);
                                    let _ = configure_tmag_wake_and_sleep(&mut tmag, wake_threshold_mt);
                                    info!(
                                        "[WINDOW] Calibration stored closed={}mT tilt={}mT open={}mT wake_thr={}mT closed_thr={}mT",
                                        next.closed_mt as u32,
                                        next.tilt_mt as u32,
                                        next.open_mt as u32,
                                        wake_threshold_mt as u32,
                                        closed_threshold_mt as u32,
                                    );
                                    SETUP_EVENT.signal(SetupEvent::CalibrationStored);
                                } else {
                                    warn!(
                                        "[WINDOW] Rejected calibration closed={}mT tilt={}mT open={}mT",
                                        next.closed_mt as u32,
                                        next.tilt_mt as u32,
                                        next.open_mt as u32,
                                    );
                                    SETUP_EVENT.signal(SetupEvent::CalibrationRejected(phase));
                                }
                            } else {
                                warn!("[WINDOW] Ignored OPEN capture without prior phases");
                                SETUP_EVENT.signal(SetupEvent::CalibrationRejected(phase));
                            }
                        }
                    }
                }
                Ok(None) => {
                    warn!("[WINDOW] Calibration {:?} did not settle", phase);
                    SETUP_EVENT.signal(SetupEvent::CalibrationRejected(phase));
                }
                Err(_) => {
                    warn!("[WINDOW] Calibration capture failed");
                    SETUP_EVENT.signal(SetupEvent::CalibrationRejected(phase));
                }
            }
            continue;
        }

        info!("[WINDOW] INT fired — field below threshold");

        // Clear interrupt latch
        let _ = tmag.read_conversion_status();

        // Quick-read to determine if field is gone or returned
        Timer::after_millis(5).await;
        let initial_sample = match read_mag_sample(&mut tmag) {
            Ok(s) => s,
            Err(_) => {
                error!("[WINDOW] I2C read failed");
                Timer::after_millis(100).await;
                continue;
            }
        };

        let initial_mag = initial_sample.magnitude_mt();

        if initial_mag > closed_threshold_mt {
            // Field is ABOVE threshold — magnet came back → CLOSED
            if current_state != WindowState::Closed {
                current_state = WindowState::Closed;
                info!("[WINDOW] State → CLOSED (mag={}mT)", initial_mag as u32);
                if let Some(gesture) = gestures.record_transition(Instant::now().as_millis()) {
                    info!("[WINDOW] Gesture → {}", gesture_str(gesture));
                    SETUP_EVENT.signal(SetupEvent::MagnetGesture(gesture));
                }
                SETUP_EVENT.signal(SetupEvent::WindowState(WindowState::Closed));
                STATE_CHANGED.signal(WindowEvent {
                    state: WindowState::Closed,
                    peak_mt_x10: (initial_mag * 10.0) as u16,
                });
            }
            // Re-arm and continue
            let _ = configure_tmag_wake_and_sleep(&mut tmag, wake_threshold_mt);
            continue;
        }

        // Field is LOW — window opening. Enter burst capture mode.
        let _ = tmag.set_mode(OperatingMode::ContinuousMeasure);
        Timer::after_millis(10).await;

        // Capture burst at ~20 Hz
        let mut burst = [classifier::MagSample {
            x_raw: 0,
            y_raw: 0,
            z_raw: 0,
        }; window_tuning::BURST_SAMPLES];

        // First sample is the initial read
        burst[0] = initial_sample;

        for slot in burst[1..].iter_mut() {
            Timer::after_millis(window_tuning::BURST_INTERVAL_MS).await;
            match read_mag_sample(&mut tmag) {
                Ok(s) => *slot = s,
                Err(_) => break,
            }
        }

        // Return to low-power wake-and-sleep mode ASAP
        let _ = configure_tmag_wake_and_sleep(&mut tmag, wake_threshold_mt);

        // Classify
        let thresholds = window_tuning::CLASSIFIER_THRESHOLDS;
        let new_state = classifier::classify_burst(&burst, &thresholds);

        // Find peak magnitude for telemetry
        let peak_mt = burst
            .iter()
            .map(|s| s.magnitude_mt())
            .fold(0.0_f32, f32::max);

        if new_state != current_state {
            current_state = new_state;
            info!(
                "[WINDOW] State → {} (peak={}mT)",
                state_str(new_state),
                peak_mt as u32
            );
            if let Some(gesture) = gestures.record_transition(Instant::now().as_millis()) {
                info!("[WINDOW] Gesture → {}", gesture_str(gesture));
                SETUP_EVENT.signal(SetupEvent::MagnetGesture(gesture));
            }
            SETUP_EVENT.signal(SetupEvent::WindowState(new_state));
            STATE_CHANGED.signal(WindowEvent {
                state: new_state,
                peak_mt_x10: (peak_mt * 10.0) as u16,
            });
        }

        // Debounce before re-arming
        Timer::after_millis(window_tuning::REARM_DELAY_MS).await;
    }
}

async fn capture_baseline_mt<I2C, D>(
    tmag: &mut Tmag5273<I2C, tmag5273::Configured, D>,
) -> Result<Option<f32>, tmag5273::Error<I2C::Error>>
where
    I2C: embedded_hal::i2c::I2c,
    D: embedded_hal::delay::DelayNs,
{
    tmag.set_mode(OperatingMode::ContinuousMeasure)?;
    Timer::after_millis(10).await;

    for _ in 0..window_tuning::CALIBRATION_MAX_ATTEMPTS {
        let stats = capture_baseline_window_mt(tmag).await?;
        if window_tuning::is_stable_calibration_window(stats.min_mt, stats.max_mt) {
            return Ok(Some(stats.average_mt));
        }
    }

    Ok(None)
}

struct CalibrationWindowStats {
    average_mt: f32,
    min_mt: f32,
    max_mt: f32,
}

async fn capture_baseline_window_mt<I2C, D>(
    tmag: &mut Tmag5273<I2C, tmag5273::Configured, D>,
) -> Result<CalibrationWindowStats, tmag5273::Error<I2C::Error>>
where
    I2C: embedded_hal::i2c::I2c,
    D: embedded_hal::delay::DelayNs,
{
    let mut total_mt = 0.0_f32;
    let mut samples = 0usize;
    let mut min_mt = f32::INFINITY;
    let mut max_mt = f32::NEG_INFINITY;

    for _ in 0..window_tuning::CALIBRATION_SAMPLES {
        let magnitude_mt = read_mag_sample(tmag)?.magnitude_mt();
        total_mt += magnitude_mt;
        min_mt = min_mt.min(magnitude_mt);
        max_mt = max_mt.max(magnitude_mt);
        samples += 1;
        Timer::after_millis(window_tuning::CALIBRATION_INTERVAL_MS).await;
    }

    Ok(CalibrationWindowStats {
        average_mt: total_mt / samples as f32,
        min_mt,
        max_mt,
    })
}

fn load_settings(settings_flash: &'static SettingsFlash) -> SocSettings {
    settings_flash.lock(|flash| load_soc_settings(&mut *flash.borrow_mut()).unwrap_or_default())
}

fn persist_settings(settings_flash: &'static SettingsFlash, settings: SocSettings) {
    settings_flash.lock(|flash| {
        let _ = save_soc_settings(&mut *flash.borrow_mut(), settings);
    });
}

fn init_tmag(
    bus: I2cDevice<
        'static,
        embassy_sync::blocking_mutex::raw::NoopRawMutex,
        embassy_nrf::twim::Twim<'static>,
    >,
) -> Result<
    Tmag5273<
        I2cDevice<
            'static,
            embassy_sync::blocking_mutex::raw::NoopRawMutex,
            embassy_nrf::twim::Twim<'static>,
        >,
        tmag5273::Configured,
        Delay,
    >,
    (),
> {
    let sensor = Tmag5273::new_with_delay(bus, DeviceVariant::B1, Delay);
    let config = TmagConfigBuilder::new()
        .operating_mode(OperatingMode::WakeUpAndSleep)
        .conversion_average(ConversionAverage::X1)
        .magnetic_channels_enabled(MagneticChannel::XYZ)
        .temp_channel_enabled(false)
        .xy_range(Range::High)
        .z_range(Range::High)
        .sleep_time(SleepTime::Ms1000)
        .build()
        .map_err(|_| ())?;

    sensor.init(&config).map_err(|_| ())
}

fn configure_tmag_wake_and_sleep<I2C, D>(
    tmag: &mut Tmag5273<I2C, tmag5273::Configured, D>,
    threshold_mt: f32,
) -> Result<(), tmag5273::Error<I2C::Error>>
where
    I2C: embedded_hal::i2c::I2c,
    D: embedded_hal::delay::DelayNs,
{
    let threshold_code = (threshold_mt * 128.0 / 80.0) as i8;
    tmag.set_thresholds(&ThresholdConfig {
        x: tmag5273::Lsb(threshold_code),
        y: tmag5273::Lsb(threshold_code),
        z: tmag5273::Lsb(threshold_code),
        temperature: tmag5273::TempThresholdConfig::DISABLED,
        hysteresis: ThresholdHysteresis::SymmetricBand,
        crossing_count: ThresholdCrossingCount::One,
        direction: MagneticThresholdDirection::Below,
    })?;
    tmag.set_interrupt(&InterruptConfig {
        mode: InterruptMode::ThroughInt,
        on_conversion_complete: false,
        on_threshold_crossing: true,
        pin_behavior: InterruptState::Latched,
        mask: false,
    })?;
    tmag.configure_wake_and_sleep()
}

fn read_mag_sample<I2C, D>(
    tmag: &mut Tmag5273<I2C, tmag5273::Configured, D>,
) -> Result<classifier::MagSample, tmag5273::Error<I2C::Error>>
where
    I2C: embedded_hal::i2c::I2c,
    D: embedded_hal::delay::DelayNs,
{
    let MagneticReading { x, y, z } = tmag.read_magnetic()?;
    Ok(classifier::MagSample {
        x_raw: mt_to_raw_lsb(x),
        y_raw: mt_to_raw_lsb(y),
        z_raw: mt_to_raw_lsb(z),
    })
}

fn mt_to_raw_lsb(value: Option<tmag5273::MilliTesla>) -> i16 {
    let mt = value.map(f32::from).unwrap_or(0.0);
    (mt * 1000.0 / 39.0625) as i16
}

fn state_str(s: WindowState) -> &'static str {
    match s {
        WindowState::Closed => "CLOSED",
        WindowState::Tilt => "TILT",
        WindowState::Open => "OPEN",
    }
}

fn gesture_str(gesture: Gesture) -> &'static str {
    match gesture {
        Gesture::Calibrate => "CALIBRATE",
        Gesture::Pairing => "PAIRING",
        Gesture::FactoryReset => "FACTORY_RESET",
    }
}
