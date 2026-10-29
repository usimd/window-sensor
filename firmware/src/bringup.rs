#![no_std]
#![no_main]

//! Board bring-up and factory test for the window-sensor.
//!
//! With a debugger attached every step runs with a timeout and prints a human summary plus
//! `MEAS key=value` / `PROMPT name` lines that scripts/factory_test.py turns into pass/fail per
//! stage. Afterwards it keeps advertising and logs VDD and the POF comparator once per second.
//!
//! Without a debugger after power-on it advertises for a few seconds (battery-holder check),
//! then soft-resets into a quiet sleep with all sensors powered down for the µA measurement.

use bt_hci::cmd::SyncCmd;
use bt_hci::cmd::le::{LeSetAdvData, LeSetAdvEnable, LeSetAdvParams};
use bt_hci::param::{
    AddrKind, AdvChannelMap, AdvFilterPolicy, AdvKind, BdAddr, Duration as HciDuration,
};
use defmt::{Format, error, info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_nrf::config::{ClockSpeed, Config, HfclkSource, LfclkSource};
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::pac::clock::vals::Lfclksrc;
use embassy_nrf::saadc::{self, ChannelConfig, Oversample, Saadc, VddInput};
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{Peri, bind_interrupts, cracen, pac, peripherals};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal::i2c::I2c;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use heapless::Vec;
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::vendor::ZephyrWriteBdAddr;
use nrf_sdc::{self as sdc, mpsl};
use static_cell::StaticCell;
use window_sensor::classifier::WindowState;
use window_sensor::partition::{PartitionLayout, SETTINGS_REGION};
use window_sensor::telemetry::{EnvReading, WindowStatus, encode_heartbeat_advertisement};
use {defmt_rtt as _, panic_probe as _};

#[path = "drivers/i2c.rs"]
mod i2c;
#[path = "bringup/sensors.rs"]
mod sensors;

const HF_TIMER_HZ: u32 = 16_000_000;
const HF_TICKS_PER_MS: u32 = HF_TIMER_HZ / 1000;
const HFXO_START_TIMEOUT_MS: u32 = 50;
const LFXO_START_TIMEOUT_MS: u32 = 2000;
const LFXO_MAX_ERROR_PPM: i32 = 100;
const SAADC_VDD_FULL_SCALE_MV: i32 = 3600;
const ADC_MAX_COUNTS: i32 = (1 << 12) - 1;
const POF_THRESHOLD_MV: u16 = 2800;
const POF_MARGIN_MV: u16 = 100;
const ADV_INTERVAL_MS: u32 = 1000;
const STANDALONE_ADV_INTERVAL_MS: u32 = 100;
const STANDALONE_ADV_WINDOW: Duration = Duration::from_secs(5);
const LED_BLINK_WINDOW: Duration = Duration::from_secs(8);
// GPREGRET[1] marker: the standalone advertising window is over, go straight to sleep.
const SLEEP_MARKER: u8 = 0xA5;

bind_interrupts!(struct Irqs {
    SERIAL20 => twim::InterruptHandler<peripherals::SERIAL20>;
    SAADC => saadc::InterruptHandler;
    SWI00 => mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => mpsl::ClockInterruptHandler;
    RADIO_0 => mpsl::HighPrioInterruptHandler;
    TIMER10 => mpsl::HighPrioInterruptHandler;
    GRTC_3 => mpsl::HighPrioInterruptHandler;
});

#[derive(Clone, Copy, Format, PartialEq, Eq)]
enum Outcome {
    Pass,
    Fail,
    Info,
}

impl Outcome {
    fn from_check(present: bool, ok: bool) -> Self {
        match (present, ok) {
            (false, _) => Outcome::Info,
            (true, true) => Outcome::Pass,
            (true, false) => Outcome::Fail,
        }
    }
}

struct Results(Vec<(&'static str, Outcome), 16>);

impl Results {
    fn record(&mut self, step: &'static str, outcome: Outcome) {
        match outcome {
            Outcome::Fail => error!("[{=str}] FAIL", step),
            _ => info!("[{=str}] {}", step, outcome),
        }
        let _ = self.0.push((step, outcome));
    }

    fn summary(&self) {
        info!("===== BRING-UP SUMMARY (pass/fail per stage: scripts/factory_test.py) =====");
        for (step, outcome) in &self.0 {
            info!("  {=str}: {}", step, outcome);
        }
        let failed = self.0.iter().filter(|(_, o)| *o == Outcome::Fail).count();
        if failed == 0 {
            info!("===== NO STEP FAILED =====");
        } else {
            error!("===== {} STEP(S) FAILED =====", failed);
        }
    }
}

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

macro_rules! ble_peripherals {
    ($p:ident) => {
        (
            mpsl::Peripherals::new(
                $p.GRTC_CH7,
                $p.GRTC_CH8,
                $p.GRTC_CH9,
                $p.GRTC_CH10,
                $p.GRTC_CH11,
                $p.TIMER10,
                $p.TIMER20,
                $p.TEMP,
                $p.PPI10_CH0,
                $p.PPI20_CH1,
                $p.PPIB11_CH0,
                $p.PPIB21_CH0,
            ),
            sdc::Peripherals::new(
                $p.PPI00_CH1,
                $p.PPI00_CH3,
                $p.PPI10_CH1,
                $p.PPI10_CH2,
                $p.PPI10_CH3,
                $p.PPI10_CH4,
                $p.PPI10_CH5,
                $p.PPI10_CH6,
                $p.PPI10_CH7,
                $p.PPI10_CH8,
                $p.PPI10_CH9,
                $p.PPI10_CH10,
                $p.PPI10_CH11,
                $p.PPIB00_CH1,
                $p.PPIB00_CH2,
                $p.PPIB00_CH3,
                $p.PPIB10_CH1,
                $p.PPIB10_CH2,
                $p.PPIB10_CH3,
            ),
            $p.CRACEN,
        )
    };
}

macro_rules! sensor_bus {
    ($p:ident) => {{
        let mut twim_config = twim::Config::default();
        twim_config.frequency = twim::Frequency::K100;
        // Internal pull-ups keep the lines defined when R1/R2 are not fitted.
        twim_config.sda_pullup = true;
        twim_config.scl_pullup = true;
        static TWIM_BUF: StaticCell<[u8; 16]> = StaticCell::new();
        i2c::TimeoutTwim::new(
            Twim::new(
                $p.SERIAL20,
                Irqs,
                $p.P1_03,
                $p.P1_02,
                twim_config,
                TWIM_BUF.init([0; 16]),
            ),
            i2c::TRANSACTION_TIMEOUT,
        )
    }};
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.clock_speed = ClockSpeed::CK128;
    // Oscillators are started and timed below, so a dead crystal cannot hang init.
    config.hfclk_source = HfclkSource::Internal;
    config.lfclk_source = LfclkSource::InternalRC;
    let mut p = embassy_nrf::init(config);

    let power_on = pac::RESET.resetreas().read().0 == 0;
    let sleep_marker = pac::POWER.gpregret(1).read().gpregret() == SLEEP_MARKER;
    pac::POWER.gpregret(1).write(|w| w.set_gpregret(0));
    if !cortex_m::peripheral::DCB::is_debugger_attached() && (power_on || sleep_marker) {
        let mut bus = sensor_bus!(p);
        sensors::all_to_sleep(&mut bus);
        drop(bus);
        if !sleep_marker {
            let (mpsl_p, sdc_p, cracen_p) = ble_peripherals!(p);
            let controller = init_ble(spawner, mpsl_p, sdc_p, cracen_p).await;
            let _ = start_advertising(controller, STANDALONE_ADV_INTERVAL_MS).await;
            Timer::after(STANDALONE_ADV_WINDOW).await;
            pac::POWER
                .gpregret(1)
                .write(|w| w.set_gpregret(SLEEP_MARKER));
            cortex_m::peripheral::SCB::sys_reset();
        }
        info!("MEAS mode=sleep");
        loop {
            Timer::after_secs(3600).await;
        }
    }

    let mut results = Results(Vec::new());
    info!(
        "[BRINGUP] window-sensor bring-up v{=str}",
        env!("CARGO_PKG_VERSION")
    );
    info!("MEAS mode=test");
    hf_timer_start();

    results.record("soc", soc_info());
    results.record("hfxo", start_hfxo());
    results.record("lfxo", lfxo_sweep());

    let mut adc = build_vdd_saadc(p.SAADC);
    adc.calibrate().await;
    let vdd_mv = sample_vdd_mv(&mut adc).await;
    results.record("vdd", vdd_check(vdd_mv));

    configure_pof();
    results.record("pof", pof_check(vdd_mv));

    results.record("rram", rram_scratch(p.RRAMC));

    let recovery = i2c::recover_bus(p.P1_02.reborrow(), p.P1_03.reborrow(), Pull::Up);
    info!("[i2c] recovery: {}", recovery);
    info!("MEAS i2c_idle={}", u8::from(recovery.idle()));
    let scl_pullup = sensors::external_pullup(p.P1_02.reborrow());
    let sda_pullup = sensors::external_pullup(p.P1_03.reborrow());
    info!("MEAS i2c_pullup_scl={}", u8::from(scl_pullup));
    info!("MEAS i2c_pullup_sda={}", u8::from(sda_pullup));
    results.record(
        "pullups",
        if scl_pullup && sda_pullup {
            Outcome::Pass
        } else {
            Outcome::Info
        },
    );

    let mut bus = sensor_bus!(p);
    let found = i2c_scan(&mut bus);
    results.record(
        "i2c",
        if recovery.idle() {
            Outcome::Pass
        } else {
            Outcome::Fail
        },
    );

    let hall = Input::new(p.P1_04, Pull::Up);
    let int1 = Input::new(p.P1_05, Pull::Down);
    let int2 = Input::new(p.P1_06, Pull::Down);
    let present = |addr| found.contains(&addr);

    let lis_ok =
        present(sensors::lis2dtw12::ADDR) && sensors::lis2dtw12::test(&mut bus, &int1, &int2).await;
    results.record(
        "lis2dtw12",
        Outcome::from_check(present(sensors::lis2dtw12::ADDR), lis_ok),
    );
    let tmag_ok =
        present(sensors::tmag5273::ADDR) && sensors::tmag5273::test(&mut bus, &hall).await;
    results.record(
        "tmag5273",
        Outcome::from_check(present(sensors::tmag5273::ADDR), tmag_ok),
    );
    let sht_ok = present(sensors::sht4x::ADDR) && sensors::sht4x::test(&mut bus).await;
    results.record(
        "sht4x",
        Outcome::from_check(present(sensors::sht4x::ADDR), sht_ok),
    );
    drop(bus);

    blink_leds(p.P1_08, p.P1_10).await;
    results.record("leds", Outcome::Info);

    let (mpsl_p, sdc_p, cracen_p) = ble_peripherals!(p);
    let controller = init_ble(spawner, mpsl_p, sdc_p, cracen_p).await;
    let ble_ok = start_advertising(controller, ADV_INTERVAL_MS).await;
    info!("MEAS ble_addr={=u64:012x}", bd_addr_u64());
    info!("MEAS ble_adv={}", u8::from(ble_ok));
    results.record("ble", if ble_ok { Outcome::Pass } else { Outcome::Fail });

    results.summary();
    info!("MEAS done=1");
    info!(
        "[monitor] advertising every {} ms; sweep the supply and watch VDD/POF",
        ADV_INTERVAL_MS
    );
    loop {
        let vdd = sample_vdd_mv(&mut adc).await;
        info!(
            "[monitor] uptime={}s vdd={}mV pof_above_{}mV={}",
            Instant::now().as_secs(),
            vdd,
            POF_THRESHOLD_MV,
            pof_above()
        );
        Timer::after_secs(1).await;
    }
}

fn soc_info() -> Outcome {
    let info = pac::FICR.info();
    let rram_kib = info.rram().read().0;
    let ram_kib = info.ram().read().0;
    let resetreas = pac::RESET.resetreas().read();
    pac::RESET.resetreas().write(|w| *w = resetreas);
    info!(
        "[soc] part=0x{:x} variant=0x{:08x} package=0x{:x} resetreas=0x{:08x}",
        info.part().read().0,
        info.variant().read().0,
        info.package().read().0,
        resetreas.0
    );
    info!(
        "MEAS device_id={=u32:08x}{=u32:08x}",
        info.deviceid(1).read(),
        info.deviceid(0).read()
    );
    info!("MEAS soc_rram_kib={}", rram_kib);
    info!("MEAS soc_ram_kib={}", ram_kib);

    let expected_rram = PartitionLayout::FLASH_SIZE / 1024;
    let expected_ram = PartitionLayout::RAM_SIZE / 1024;
    if rram_kib != expected_rram || ram_kib != expected_ram {
        warn!(
            "[soc] FICR memory differs from partition.rs (rram {}KiB, ram {}KiB)",
            expected_rram, expected_ram
        );
        return Outcome::Fail;
    }
    Outcome::Pass
}

fn hf_timer_start() {
    let t = pac::TIMER21;
    t.tasks_stop().write_value(1);
    t.mode()
        .write(|w| w.set_mode(pac::timer::vals::Mode::Timer));
    t.bitmode()
        .write(|w| w.set_bitmode(pac::timer::vals::Bitmode::_32bit));
    t.prescaler().write(|w| w.set_prescaler(0));
    t.tasks_clear().write_value(1);
    t.tasks_start().write_value(1);
}

fn hf_ticks() -> u32 {
    let t = pac::TIMER21;
    t.tasks_capture(0).write_value(1);
    t.cc(0).read()
}

/// Busy-waits on the HF timer (GRTC may be stalled while LFCLK is switched); returns elapsed ms.
fn wait_until(mut done: impl FnMut() -> bool, timeout_ms: u32) -> Option<u32> {
    let start = hf_ticks();
    loop {
        let elapsed = hf_ticks().wrapping_sub(start);
        if done() {
            return Some(elapsed / HF_TICKS_PER_MS);
        }
        if elapsed > timeout_ms * HF_TICKS_PER_MS {
            return None;
        }
    }
}

fn start_hfxo() -> Outcome {
    let clock = pac::CLOCK;
    clock.events_xostarted().write_value(0);
    clock.tasks_xostart().write_value(1);
    let started = wait_until(
        || clock.events_xostarted().read() != 0,
        HFXO_START_TIMEOUT_MS,
    );
    info!("MEAS hfxo_ok={}", u8::from(started.is_some()));
    match started {
        Some(ms) => {
            info!("MEAS hfxo_ms={}", ms);
            Outcome::Pass
        }
        None => {
            error!("[hfxo] no XOSTARTED within {} ms", HFXO_START_TIMEOUT_MS);
            Outcome::Fail
        }
    }
}

fn lfclk_switch(src: Lfclksrc, intcap: Option<u8>) -> Option<u32> {
    let clock = pac::CLOCK;
    clock.tasks_lfclkstop().write_value(1);
    let _ = wait_until(|| !clock.lfclk().stat().read().state(), 10);
    if let Some(val) = intcap {
        pac::OSCILLATORS
            .xosc32ki()
            .intcap()
            .write(|w| w.set_val(val));
    }
    clock.lfclk().src().write(|w| w.set_src(src));
    clock.events_lfclkstarted().write_value(0);
    clock.tasks_lfclkstart().write_value(1);
    let ms = wait_until(
        || clock.events_lfclkstarted().read() != 0,
        LFXO_START_TIMEOUT_MS,
    )?;
    let stat = clock.lfclk().stat().read();
    (stat.state() && stat.src() == src).then_some(ms)
}

/// LFCLK error vs HFXO in ppm, measured as GRTC (LFCLK) time against TIMER21 (HFXO) over 1 s.
fn lfclk_ppm() -> Option<i32> {
    let guard = hf_ticks();
    let first = Instant::now();
    let mut a = first;
    while a == first {
        a = Instant::now();
        if hf_ticks().wrapping_sub(guard) > 10 * HF_TICKS_PER_MS {
            return None;
        }
    }
    let h0 = hf_ticks();
    let target = a + Duration::from_secs(1);
    let mut b = a;
    while b < target {
        b = Instant::now();
        if hf_ticks().wrapping_sub(h0) > 3 * HF_TIMER_HZ {
            return None;
        }
    }
    let h1 = hf_ticks();
    let lf_us = (b - a).as_micros() as i64;
    let hf_us = i64::from(h1.wrapping_sub(h0)) * 1_000_000 / i64::from(HF_TIMER_HZ);
    Some(((lf_us - hf_us) * 1_000_000 / hf_us) as i32)
}

/// INTCAP code for a total LFXO load capacitance, per Zephyr soc/nordic/nrf54l/soc.c.
fn lfxo_intcap(load_ff: u32) -> u8 {
    let trim = pac::FICR.xosc32ktrim().read();
    let slope = i32::from(trim.slope() ^ 0x100) - 0x100;
    let offset = i32::from(trim.offset());
    let mid = (2 * load_ff as i32 - 12_000) * (slope + 392) + (offset << 3) * 1000;
    ((mid + 256_000) / 512_000) as u8
}

fn lfxo_sweep() -> Outcome {
    match lfclk_ppm() {
        // LFRC is only ~±2% accurate uncalibrated; ~0 ppm here means GRTC is not tracking LFCLK.
        Some(ppm) => info!("MEAS lfrc_ppm={}", ppm),
        None => warn!("[lfxo] GRTC not advancing on LFRC"),
    }

    // CC8V-T1A is sold with 7, 9 and 12.5 pF load capacitance.
    let loads_ff: [u32; 3] = [7_000, 9_000, 12_500];
    let mut best: Option<(u32, i32)> = None;
    for load_ff in loads_ff {
        let code = lfxo_intcap(load_ff);
        match lfclk_switch(Lfclksrc::Lfxo, Some(code)) {
            None => warn!(
                "[lfxo] {} fF (intcap {}): did not start within {} ms",
                load_ff, code, LFXO_START_TIMEOUT_MS
            ),
            Some(ms) => match lfclk_ppm() {
                None => warn!(
                    "[lfxo] {} fF (intcap {}): started in {} ms but GRTC stalled",
                    load_ff, code, ms
                ),
                Some(ppm) => {
                    info!(
                        "[lfxo] {} fF (intcap {}): started in {} ms, {} ppm vs HFXO",
                        load_ff, code, ms, ppm
                    );
                    if best.is_none_or(|(_, b)| ppm.abs() < b.abs()) {
                        best = Some((load_ff, ppm));
                    }
                }
            },
        }
    }

    if lfclk_switch(Lfclksrc::Lfrc, None).is_none() {
        error!("[lfxo] could not switch back to LFRC");
    }

    info!("MEAS lfxo_ok={}", u8::from(best.is_some()));
    let Some((load_ff, ppm)) = best else {
        return Outcome::Fail;
    };
    info!("MEAS lfxo_best_ff={}", load_ff);
    info!("MEAS lfxo_ppm={}", ppm);
    if ppm.abs() <= LFXO_MAX_ERROR_PPM {
        Outcome::Pass
    } else {
        Outcome::Fail
    }
}

fn build_vdd_saadc(saadc: Peri<'static, peripherals::SAADC>) -> Saadc<'static, 1> {
    let mut channel = ChannelConfig::single_ended(VddInput);
    channel.time = saadc::Time::_40US;
    let mut config = saadc::Config::default();
    config.resolution = saadc::Resolution::_12bit;
    config.oversample = Oversample::Over8x;
    Saadc::new(saadc, Irqs, config, [channel])
}

async fn sample_vdd_mv(adc: &mut Saadc<'static, 1>) -> u16 {
    let mut buf = [0i16; 1];
    adc.sample(&mut buf).await;
    let mv = i32::from(buf[0].max(0)) * SAADC_VDD_FULL_SCALE_MV / ADC_MAX_COUNTS;
    mv as u16
}

fn vdd_check(vdd_mv: u16) -> Outcome {
    info!("MEAS vdd_mv={}", vdd_mv);
    if (1700..=3700).contains(&vdd_mv) {
        Outcome::Pass
    } else {
        Outcome::Fail
    }
}

fn configure_pof() {
    pac::REGULATORS.pofcon().write(|w| {
        w.set_pof(true);
        w.set_threshold(pac::regulators::vals::Threshold::V28);
        w.set_eventdisable(pac::regulators::vals::Eventdisable::Enabled);
    });
}

fn pof_above() -> bool {
    matches!(
        pac::REGULATORS.pofstat().read().comparator(),
        pac::regulators::vals::Comparator::Above
    )
}

fn pof_check(vdd_mv: u16) -> Outcome {
    let above = pof_above();
    info!("MEAS pof_above={}", u8::from(above));
    if vdd_mv >= POF_THRESHOLD_MV + POF_MARGIN_MV {
        if above { Outcome::Pass } else { Outcome::Fail }
    } else if vdd_mv + POF_MARGIN_MV <= POF_THRESHOLD_MV {
        if above { Outcome::Fail } else { Outcome::Pass }
    } else {
        Outcome::Info
    }
}

fn rram_scratch(rramc: Peri<'static, peripherals::RRAMC>) -> Outcome {
    const PAGE: u32 = 4096;
    let mut flash = Nvmc::new(rramc);
    let page = SETTINGS_REGION.end() - PAGE;
    let mut pattern = [0u8; 32];
    for (i, b) in pattern.iter_mut().enumerate() {
        *b = (i as u8).wrapping_mul(37) ^ 0xA5;
    }
    let mut readback = [0u8; 32];

    let written = flash.erase(page, page + PAGE).is_ok()
        && flash.write(page, &pattern).is_ok()
        && flash.read(page, &mut readback).is_ok()
        && readback == pattern;
    let erased = flash.erase(page, page + PAGE).is_ok()
        && flash.read(page, &mut readback).is_ok()
        && readback.iter().all(|b| *b == 0xff);

    info!("[rram] scratch page 0x{:08x}", page);
    info!("MEAS rram_ok={}", u8::from(written && erased));
    if written && erased {
        Outcome::Pass
    } else {
        Outcome::Fail
    }
}

fn i2c_scan(bus: &mut i2c::TimeoutTwim<'_>) -> Vec<u8, 16> {
    let mut found: Vec<u8, 16> = Vec::new();
    for addr in 0x08..=0x77u8 {
        let mut byte = [0u8; 1];
        if bus.read(addr, &mut byte).is_ok() {
            let _ = found.push(addr);
        }
    }
    info!("[i2c] ACK from {=[u8]:#04x}", found[..]);
    info!("MEAS i2c_ack_19={}", u8::from(found.contains(&0x19)));
    info!("MEAS i2c_ack_22={}", u8::from(found.contains(&0x22)));
    info!("MEAS i2c_ack_44={}", u8::from(found.contains(&0x44)));
    found
}

async fn blink_leds(
    blue: Peri<'static, peripherals::P1_08>,
    red: Peri<'static, peripherals::P1_10>,
) {
    let mut blue = Output::new(blue, Level::Low, OutputDrive::Standard);
    let mut red = Output::new(red, Level::Low, OutputDrive::Standard);
    info!("PROMPT leds");
    let deadline = Instant::now() + LED_BLINK_WINDOW;
    while Instant::now() < deadline {
        blue.set_high();
        red.set_low();
        Timer::after_millis(250).await;
        blue.set_low();
        red.set_high();
        Timer::after_millis(250).await;
    }
    red.set_low();
}

async fn init_ble(
    spawner: Spawner,
    mpsl_p: mpsl::Peripherals<'static>,
    sdc_p: sdc::Peripherals<'static>,
    cracen_p: Peri<'static, peripherals::CRACEN>,
) -> &'static sdc::SoftdeviceController<'static> {
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(unwrap!(MultiprotocolServiceLayer::new(
        mpsl_p, Irqs, lfclk_cfg
    )));
    spawner.spawn(unwrap!(mpsl_task(&*mpsl)));

    static RNG: StaticCell<cracen::Cracen<'static, embassy_nrf::mode::Blocking>> =
        StaticCell::new();
    let rng = RNG.init(cracen::Cracen::new_blocking(cracen_p));
    static SDC_MEM: StaticCell<sdc::Mem<4096>> = StaticCell::new();
    static SDC: StaticCell<sdc::SoftdeviceController> = StaticCell::new();
    let sdc_builder = unwrap!(sdc::Builder::new()).support_adv();
    let sdc_builder = unwrap!(sdc_builder.adv_count(1));
    let sdc_builder = unwrap!(sdc_builder.adv_buffer_cfg(31));
    let controller = SDC.init(unwrap!(sdc_builder.build(
        sdc_p,
        rng,
        mpsl,
        SDC_MEM.init(sdc::Mem::new())
    )));
    if ZephyrWriteBdAddr::new(bd_addr())
        .exec(controller)
        .await
        .is_err()
    {
        error!("[ble] setting the device address failed");
    }
    controller
}

fn bd_addr_u64() -> u64 {
    let ficr = pac::FICR;
    let high = u64::from(ficr.deviceaddr(1).read());
    ((high << 32) | u64::from(ficr.deviceaddr(0).read()) | 0x0000_c000_0000_0000)
        & 0x0000_ffff_ffff_ffff
}

fn bd_addr() -> BdAddr {
    BdAddr::new(unwrap!(bd_addr_u64().to_le_bytes()[..6].try_into()))
}

async fn start_advertising(
    controller: &sdc::SoftdeviceController<'static>,
    interval_ms: u32,
) -> bool {
    let mut payload = [0u8; 31];
    let env = EnvReading {
        temperature_cdeg: 0,
        humidity_cpct: 0,
        battery_soc_pct: 100,
    };
    let status = WindowStatus {
        state: WindowState::Closed,
        tampered: false,
        problem: false,
    };
    let len = encode_heartbeat_advertisement(&mut payload, 0, env, status);
    let mut adv_data = [0u8; 31];
    adv_data[..len].copy_from_slice(&payload[..len]);

    let params = LeSetAdvParams::new(
        HciDuration::from_millis(interval_ms),
        HciDuration::from_millis(interval_ms),
        AdvKind::AdvNonconnInd,
        AddrKind::PUBLIC,
        AddrKind::PUBLIC,
        BdAddr::default(),
        AdvChannelMap::ALL,
        AdvFilterPolicy::default(),
    );
    params.exec(controller).await.is_ok()
        && LeSetAdvData::new(len as u8, adv_data)
            .exec(controller)
            .await
            .is_ok()
        && LeSetAdvEnable::new(true).exec(controller).await.is_ok()
}
