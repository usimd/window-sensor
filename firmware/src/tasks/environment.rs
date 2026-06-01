//! Environment monitoring task — periodic SHT4x + LIS2DTW12 temp sampling.
//!
//! Runs every 5 minutes by default and sends readings to the BLE task via signal.
//! Battery refresh is opportunistic and low-duty: a weekly default SAADC VDD
//! measurement with an optional short SHT4x heater-assisted load pulse when the
//! `soc-heater` feature is enabled.

use defmt::*;
#[cfg(test)]
use embassy_futures::block_on;
use embassy_time::{Duration, Timer};

use crate::DIAGNOSTICS_STATE;
use crate::ENV_DATA;
#[cfg(feature = "soc-heater")]
use crate::I2cBus;
use crate::SettingsFlash;
use crate::SharedI2cBus;

#[cfg(feature = "soc-heater")]
use window_sensor::battery::lowest_resistance_measurement;
use window_sensor::battery::{BatteryEstimator, BatteryMeasurement, DEFAULT_HEATER_CURRENT_MA};
#[cfg(feature = "soc-heater")]
use window_sensor::battery::{
    HEATER_CURRENT_20MW_MA, HEATER_CURRENT_110MW_MA, HEATER_CURRENT_200MW_MA,
};
use window_sensor::settings::{SocSettings, load_soc_settings, save_soc_settings};
pub use window_sensor::telemetry::EnvReading;

use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_nrf::pac;
use embassy_nrf::saadc::{self, ChannelConfig, Oversample, Saadc, VddInput};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex as AsyncMutex;
use lis2dtw12::Lis2dtw12;
use lis2dtw12::interface::{I2CInterface as LisI2cInterface, SlaveAddr as LisSlaveAddr};
#[cfg(feature = "soc-heater")]
use sensirion_i2c::i2c as sensirion_i2c;
#[cfg(feature = "soc-heater")]
use sht4x::{Error as ShtError, HeatingDuration, HeatingPower, SensorData};
use sht4x::{Precision as ShtPrecision, Sht4x};

const SHT4X_ADDR: u8 = 0x44;
const ADC_MAX_COUNTS: i32 = (1 << 12) - 1;
const SAADC_VDD_FULL_SCALE_MV: i32 = 3600;
const INITIAL_SOC_REFRESH_CYCLES: u32 = 12;
#[cfg(feature = "soc-heater")]
const INITIAL_SOC_AUTOCAL_SAMPLES: usize = 3;

#[derive(Clone, Copy)]
pub struct EnvironmentConfig {
    pub env_period: Duration,
    pub soc_refresh_cycles: u32,
    pub sht_precision: ShtPrecision,
    #[cfg(feature = "soc-heater")]
    pub heater_power: HeatingPower,
    #[cfg(feature = "soc-heater")]
    pub heater_duration: HeatingDuration,
}

impl EnvironmentConfig {
    pub const fn default_env_period() -> Duration {
        Duration::from_secs(300)
    }

    pub const fn default_soc_refresh_cycles() -> u32 {
        (7 * 24 * 60 * 60) / 300
    }
}

impl Default for EnvironmentConfig {
    fn default() -> Self {
        Self {
            env_period: Self::default_env_period(),
            soc_refresh_cycles: Self::default_soc_refresh_cycles(),
            sht_precision: ShtPrecision::High,
            #[cfg(feature = "soc-heater")]
            heater_power: HeatingPower::Medium,
            #[cfg(feature = "soc-heater")]
            heater_duration: HeatingDuration::Short,
        }
    }
}

#[embassy_executor::task]
pub async fn environment_task(
    bus: &'static SharedI2cBus,
    saadc: &'static AsyncMutex<NoopRawMutex, Saadc<'static, 1>>,
    settings_flash: &'static SettingsFlash,
    config: EnvironmentConfig,
) {
    info!("[ENV] Task started");

    let mut cycle: u32 = 0;
    let mut stored = load_settings(settings_flash);
    let mut battery =
        BatteryEstimator::from_state(stored.last_battery_percent, stored.best_resistance_mohm);
    let mut first_refresh_pending = !stored.autocal_complete;

    loop {
        match read_environment(
            bus,
            saadc,
            config,
            cycle,
            &mut battery,
            first_refresh_pending,
        )
        .await
        {
            Ok(reading) => {
                ENV_DATA.signal(reading);
                info!(
                    "[ENV] T={}cdeg RH={}cpct SoC={}pct",
                    reading.temperature_cdeg, reading.humidity_cpct, reading.battery_soc_pct
                );
                stored.autocal_complete = true;
                stored.last_battery_percent = battery.current_percent();
                stored.best_resistance_mohm = battery.best_resistance_mohm();
                persist_settings(settings_flash, stored);
                first_refresh_pending = false;
            }
            Err(_) => warn!("[ENV] Sample failed"),
        }

        cycle = cycle.wrapping_add(1);
        Timer::after(config.env_period).await;
    }
}

async fn read_environment(
    bus: &'static SharedI2cBus,
    saadc: &'static AsyncMutex<NoopRawMutex, Saadc<'static, 1>>,
    config: EnvironmentConfig,
    cycle: u32,
    battery: &mut BatteryEstimator,
    first_refresh_pending: bool,
) -> Result<EnvReading, ()> {
    let mut sht = Sht4x::<_, embassy_time::Delay>::new(I2cDevice::new(bus));
    let mut delay = embassy_time::Delay;
    let measurement = sht
        .measure(config.sht_precision, &mut delay)
        .map_err(|_| ())?;

    let lis_bus = LisI2cInterface::new(I2cDevice::new(bus), LisSlaveAddr::Alternative(true));
    let mut lis = Lis2dtw12::new(lis_bus);
    let board_temp_c = lis.get_temperature().ok();

    let ambient_temp_cdeg = (measurement.temperature_milli_celsius() / 10) as i16;
    let refresh_due = is_soc_refresh_due(config, cycle, first_refresh_pending);
    #[cfg(feature = "soc-heater")]
    let soc_ctx = SocRefreshContext {
        config,
        ambient_temp_cdeg,
        board_temp_c,
        auto_calibrate: first_refresh_pending,
    };

    #[cfg(feature = "soc-heater")]
    let battery_soc_pct =
        estimate_battery_soc_pct_with_heater(saadc, bus, soc_ctx, battery, refresh_due).await;

    #[cfg(not(feature = "soc-heater"))]
    let battery_soc_pct = estimate_battery_soc_pct(
        saadc,
        bus,
        config,
        ambient_temp_cdeg,
        board_temp_c,
        battery,
        refresh_due,
    )
    .await;

    #[cfg(feature = "soc-heater")]
    if battery_soc_pct == 0 {
        warn!("[ENV] Heater-assisted SoC estimate hit empty-cell floor");
    }

    Ok(EnvReading {
        temperature_cdeg: ambient_temp_cdeg,
        humidity_cpct: (measurement.humidity_milli_percent() / 10) as u16,
        battery_soc_pct,
    })
}

#[cfg(feature = "soc-heater")]
#[derive(Clone, Copy)]
struct DeferredHeaterMeasurement {
    wait: Duration,
}

#[cfg(feature = "soc-heater")]
#[derive(Clone, Copy)]
struct SocRefreshContext {
    config: EnvironmentConfig,
    ambient_temp_cdeg: i16,
    board_temp_c: Option<f32>,
    auto_calibrate: bool,
}

#[cfg(feature = "soc-heater")]
fn start_sht_heater_measurement(
    bus: &mut I2cDevice<'static, embassy_sync::blocking_mutex::raw::NoopRawMutex, I2cBus>,
    power: HeatingPower,
    duration: HeatingDuration,
) -> Result<
    DeferredHeaterMeasurement,
    ShtError<embassy_embedded_hal::shared_bus::I2cDeviceError<embassy_nrf::twim::Error>>,
> {
    let command = heater_command(power, duration);
    sensirion_i2c::write_command_u8(bus, SHT4X_ADDR, command).map_err(ShtError::I2c)?;

    Ok(DeferredHeaterMeasurement {
        wait: heater_wait_duration(duration),
    })
}

#[cfg(feature = "soc-heater")]
fn finish_sht_heater_measurement(
    bus: &mut I2cDevice<'static, embassy_sync::blocking_mutex::raw::NoopRawMutex, I2cBus>,
    deferred: DeferredHeaterMeasurement,
) -> Result<
    SensorData,
    ShtError<embassy_embedded_hal::shared_bus::I2cDeviceError<embassy_nrf::twim::Error>>,
> {
    let _ = deferred;
    let mut response = [0; 6];
    sensirion_i2c::read_words_with_crc(bus, SHT4X_ADDR, &mut response).map_err(ShtError::from)?;

    Ok(SensorData {
        temperature: u16::from_be_bytes([response[0], response[1]]),
        humidity: u16::from_be_bytes([response[3], response[4]]),
    })
}

#[cfg(feature = "soc-heater")]
async fn heat_sht_and_measure(
    bus: &'static SharedI2cBus,
    power: HeatingPower,
    duration: HeatingDuration,
) -> Result<
    SensorData,
    ShtError<embassy_embedded_hal::shared_bus::I2cDeviceError<embassy_nrf::twim::Error>>,
> {
    let deferred = {
        let mut device = I2cDevice::new(bus);
        start_sht_heater_measurement(&mut device, power, duration)?
    };

    Timer::after(deferred.wait).await;

    let mut device = I2cDevice::new(bus);
    finish_sht_heater_measurement(&mut device, deferred)
}

#[cfg(feature = "soc-heater")]
const fn heater_command(power: HeatingPower, duration: HeatingDuration) -> u8 {
    match (power, duration) {
        (HeatingPower::Low, HeatingDuration::Long) => 0x1e,
        (HeatingPower::Low, HeatingDuration::Short) => 0x15,
        (HeatingPower::Medium, HeatingDuration::Long) => 0x2f,
        (HeatingPower::Medium, HeatingDuration::Short) => 0x24,
        (HeatingPower::High, HeatingDuration::Long) => 0x39,
        (HeatingPower::High, HeatingDuration::Short) => 0x32,
    }
}

#[cfg(feature = "soc-heater")]
const fn heater_wait_duration(duration: HeatingDuration) -> Duration {
    match duration {
        HeatingDuration::Short => Duration::from_millis(110),
        HeatingDuration::Long => Duration::from_millis(1100),
    }
}

async fn estimate_battery_soc_pct(
    saadc: &'static AsyncMutex<NoopRawMutex, Saadc<'static, 1>>,
    bus: &'static SharedI2cBus,
    config: EnvironmentConfig,
    ambient_temp_cdeg: i16,
    board_temp_c: Option<f32>,
    battery: &mut BatteryEstimator,
    refresh_due: bool,
) -> u8 {
    if !refresh_due {
        return battery.current_percent();
    }

    let measured_open_mv = sample_vdd_mv(saadc).await;

    let _ = (bus, config);

    if let Some(open_mv) = measured_open_mv {
        return battery.update_from_open_voltage(open_mv).percent;
    }

    let open_mv = estimate_open_circuit_mv(ambient_temp_cdeg, board_temp_c);
    let load_mv = open_mv.saturating_sub(estimate_loaded_voltage_drop_mv(
        ambient_temp_cdeg,
        board_temp_c,
    ));

    battery
        .update_from_measurement(BatteryMeasurement {
            open_mv,
            load_mv,
            load_current_ma: DEFAULT_HEATER_CURRENT_MA,
        })
        .percent
}

#[cfg(feature = "soc-heater")]
async fn estimate_battery_soc_pct_with_heater(
    saadc: &'static AsyncMutex<NoopRawMutex, Saadc<'static, 1>>,
    bus: &'static SharedI2cBus,
    ctx: SocRefreshContext,
    battery: &mut BatteryEstimator,
    refresh_due: bool,
) -> u8 {
    if !refresh_due {
        return battery.current_percent();
    }

    if let Some(measurement) = measure_battery_with_heater(saadc, bus, ctx).await {
        return battery.update_from_measurement(measurement).percent;
    }

    estimate_battery_soc_pct(
        saadc,
        bus,
        ctx.config,
        ctx.ambient_temp_cdeg,
        ctx.board_temp_c,
        battery,
        true,
    )
    .await
}

#[cfg(feature = "soc-heater")]
async fn measure_battery_with_heater(
    saadc: &'static AsyncMutex<NoopRawMutex, Saadc<'static, 1>>,
    bus: &'static SharedI2cBus,
    ctx: SocRefreshContext,
) -> Option<BatteryMeasurement> {
    if ctx.auto_calibrate {
        return measure_battery_with_auto_calibration(saadc, bus, ctx).await;
    }

    measure_single_battery_pulse(saadc, bus, ctx).await
}

#[cfg(feature = "soc-heater")]
async fn measure_battery_with_auto_calibration(
    saadc: &'static AsyncMutex<NoopRawMutex, Saadc<'static, 1>>,
    bus: &'static SharedI2cBus,
    ctx: SocRefreshContext,
) -> Option<BatteryMeasurement> {
    let mut samples = [None; INITIAL_SOC_AUTOCAL_SAMPLES];

    for slot in &mut samples {
        *slot = measure_single_battery_pulse(saadc, bus, ctx).await;
        Timer::after_millis(250).await;
    }

    let mut collected = [BatteryMeasurement {
        open_mv: 0,
        load_mv: 0,
        load_current_ma: DEFAULT_HEATER_CURRENT_MA,
    }; INITIAL_SOC_AUTOCAL_SAMPLES];
    let mut count = 0;

    for sample in samples.into_iter().flatten() {
        collected[count] = sample;
        count += 1;
    }

    lowest_resistance_measurement(&collected[..count])
}

#[cfg(feature = "soc-heater")]
async fn measure_single_battery_pulse(
    saadc: &'static AsyncMutex<NoopRawMutex, Saadc<'static, 1>>,
    bus: &'static SharedI2cBus,
    ctx: SocRefreshContext,
) -> Option<BatteryMeasurement> {
    let open_mv = sample_vdd_mv(saadc).await?;

    let deferred = {
        let mut device = I2cDevice::new(bus);
        start_sht_heater_measurement(
            &mut device,
            ctx.config.heater_power,
            ctx.config.heater_duration,
        )
        .ok()?
    };

    let load_wait = deferred.wait / 2;
    Timer::after(load_wait).await;
    let load_mv = sample_vdd_mv(saadc).await.unwrap_or_else(|| {
        open_mv.saturating_sub(estimate_loaded_voltage_drop_mv(
            ctx.ambient_temp_cdeg,
            ctx.board_temp_c,
        ))
    });

    let remaining_wait = Duration::from_ticks(
        deferred
            .wait
            .as_ticks()
            .saturating_sub(load_wait.as_ticks()),
    );
    Timer::after(remaining_wait).await;

    let mut device = I2cDevice::new(bus);
    let _ = finish_sht_heater_measurement(&mut device, deferred);

    Some(BatteryMeasurement {
        open_mv,
        load_mv,
        load_current_ma: heater_current_ma(ctx.config.heater_power),
    })
}

async fn sample_vdd_mv(saadc: &'static AsyncMutex<NoopRawMutex, Saadc<'static, 1>>) -> Option<u16> {
    let mut sample = [0i16; 1];
    let mut adc = saadc.lock().await;
    adc.sample(&mut sample).await;
    counts_to_vdd_mv(sample[0])
}

fn counts_to_vdd_mv(raw: i16) -> Option<u16> {
    if raw <= 0 {
        return None;
    }

    let mv = (i32::from(raw) * SAADC_VDD_FULL_SCALE_MV) / ADC_MAX_COUNTS;
    u16::try_from(mv).ok()
}

fn is_soc_refresh_due(config: EnvironmentConfig, cycle: u32, first_refresh_pending: bool) -> bool {
    let refresh_cycles = if first_refresh_pending {
        INITIAL_SOC_REFRESH_CYCLES.min(config.soc_refresh_cycles.max(1))
    } else {
        config.soc_refresh_cycles.max(1)
    };

    cycle == 0 || cycle.is_multiple_of(refresh_cycles)
}

fn load_settings(settings_flash: &'static SettingsFlash) -> SocSettings {
    settings_flash.lock(|flash| {
        let mut flash = flash.borrow_mut();
        load_soc_settings(&mut *flash).unwrap_or_default()
    })
}

fn persist_settings(settings_flash: &'static SettingsFlash, settings: SocSettings) {
    if power_fail_warning_active() {
        DIAGNOSTICS_STATE.lock(|state| state.borrow_mut().note_pofwarn());
        warn!("[ENV] Skipping SoC settings write during POFWARN");
        return;
    }

    settings_flash.lock(|flash| {
        let mut flash = flash.borrow_mut();
        if save_soc_settings(&mut *flash, settings).is_err() {
            warn!("[ENV] Failed to persist SoC settings");
        }
    });
}

fn power_fail_warning_active() -> bool {
    matches!(
        pac::REGULATORS.pofstat().read().comparator(),
        pac::regulators::vals::Comparator::BELOW
    )
}

#[cfg(feature = "soc-heater")]
const fn heater_current_ma(power: HeatingPower) -> u16 {
    match power {
        HeatingPower::Low => HEATER_CURRENT_20MW_MA,
        HeatingPower::Medium => HEATER_CURRENT_110MW_MA,
        HeatingPower::High => HEATER_CURRENT_200MW_MA,
    }
}

pub fn build_vdd_saadc(
    saadc: embassy_nrf::Peri<'static, embassy_nrf::peripherals::SAADC>,
    irq: impl embassy_nrf::interrupt::typelevel::Binding<
        embassy_nrf::interrupt::typelevel::SAADC,
        saadc::InterruptHandler,
    > + 'static,
) -> Saadc<'static, 1> {
    let mut channel = ChannelConfig::single_ended(VddInput);
    channel.time = saadc::Time::_40US;

    let mut config = saadc::Config::default();
    config.resolution = saadc::Resolution::_12BIT;
    config.oversample = Oversample::OVER8X;

    Saadc::new(saadc, irq, config, [channel])
}

fn estimate_open_circuit_mv(ambient_temp_cdeg: i16, board_temp_c: Option<f32>) -> u16 {
    let cold_penalty_mv = if ambient_temp_cdeg < 0 { 80 } else { 0 };
    let warm_penalty_mv = match board_temp_c {
        Some(temp_c) if temp_c > 40.0 => 40,
        _ => 0,
    };

    3600u16.saturating_sub(cold_penalty_mv + warm_penalty_mv)
}

fn estimate_loaded_voltage_drop_mv(ambient_temp_cdeg: i16, board_temp_c: Option<f32>) -> u16 {
    let base_mv = 280;
    let cold_extra_mv = if ambient_temp_cdeg < 0 { 170 } else { 0 };
    let hot_extra_mv = match board_temp_c {
        Some(temp_c) if temp_c > 40.0 => 100,
        _ => 0,
    };

    base_mv + cold_extra_mv + hot_extra_mv
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn soc_is_stable_between_refresh_cycles() {
        let mut battery = BatteryEstimator::new(87);
        assert_eq!(
            block_on(estimate_battery_soc_pct_placeholder(
                2150,
                Some(24.0),
                &mut battery,
                false,
            )),
            87
        );
        assert_eq!(
            block_on(estimate_battery_soc_pct_placeholder(
                2150,
                Some(24.0),
                &mut battery,
                false,
            )),
            87
        );
    }

    #[test]
    fn soc_refresh_is_monotonic_and_penalizes_cold_cell() {
        let mut battery = BatteryEstimator::new(100);
        assert_eq!(
            block_on(estimate_battery_soc_pct_placeholder(
                -500,
                Some(20.0),
                &mut battery,
                true,
            )),
            90
        );
    }

    #[test]
    fn soc_refresh_penalizes_hot_board() {
        let mut battery = BatteryEstimator::new(88);
        assert_eq!(
            block_on(estimate_battery_soc_pct_placeholder(
                2200,
                Some(45.0),
                &mut battery,
                true,
            )),
            78
        );
    }

    #[test]
    fn first_refresh_happens_well_before_weekly_cadence() {
        let mut battery = BatteryEstimator::new(100);
        assert_eq!(
            block_on(estimate_battery_soc_pct_placeholder(
                2200,
                Some(24.0),
                &mut battery,
                true,
            )),
            90
        );
    }

    async fn estimate_battery_soc_pct_placeholder(
        ambient_temp_cdeg: i16,
        board_temp_c: Option<f32>,
        battery: &mut BatteryEstimator,
        refresh_due: bool,
    ) -> u8 {
        if !refresh_due {
            return battery.current_percent();
        }

        let open_mv = estimate_open_circuit_mv(ambient_temp_cdeg, board_temp_c);
        let load_mv = open_mv.saturating_sub(estimate_loaded_voltage_drop_mv(
            ambient_temp_cdeg,
            board_temp_c,
        ));

        battery
            .update_from_measurement(BatteryMeasurement {
                open_mv,
                load_mv,
                load_current_ma: DEFAULT_HEATER_CURRENT_MA,
            })
            .percent
    }

    #[cfg(feature = "soc-heater")]
    #[test]
    fn heater_command_matches_datasheet_codes() {
        assert_eq!(
            heater_command(HeatingPower::Low, HeatingDuration::Long),
            0x1e
        );
        assert_eq!(
            heater_command(HeatingPower::Low, HeatingDuration::Short),
            0x15
        );
        assert_eq!(
            heater_command(HeatingPower::Medium, HeatingDuration::Long),
            0x2f
        );
        assert_eq!(
            heater_command(HeatingPower::Medium, HeatingDuration::Short),
            0x24
        );
        assert_eq!(
            heater_command(HeatingPower::High, HeatingDuration::Long),
            0x39
        );
        assert_eq!(
            heater_command(HeatingPower::High, HeatingDuration::Short),
            0x32
        );
    }

    #[cfg(feature = "soc-heater")]
    #[test]
    fn heater_wait_duration_matches_sensor_timing_budget() {
        assert_eq!(
            heater_wait_duration(HeatingDuration::Short),
            Duration::from_millis(110)
        );
        assert_eq!(
            heater_wait_duration(HeatingDuration::Long),
            Duration::from_millis(1100)
        );
    }

    #[test]
    fn default_environment_config_matches_requirement_defaults() {
        let config = EnvironmentConfig::default();
        assert_eq!(config.env_period, Duration::from_secs(300));
        assert_eq!(config.soc_refresh_cycles, 2016);

        #[cfg(feature = "soc-heater")]
        {
            assert!(matches!(config.heater_power, HeatingPower::Medium));
            assert!(matches!(config.heater_duration, HeatingDuration::Short));
        }
    }

    #[test]
    fn vdd_counts_convert_to_mv() {
        assert_eq!(counts_to_vdd_mv(2048), Some(1800));
        assert_eq!(counts_to_vdd_mv(1024), Some(900));
        assert_eq!(counts_to_vdd_mv(0), None);
    }
}
