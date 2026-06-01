use bt_hci::cmd::SyncCmd;
use core::cell::RefCell;
use defmt::*;
use embassy_executor::Spawner;

use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_nrf::cracen;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::nvmc;
use embassy_nrf::pac;
use embassy_nrf::saadc;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex as AsyncMutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use lis2dtw12::Lis2dtw12;
use lis2dtw12::interface::{I2CInterface as LisI2cInterface, SlaveAddr as LisSlaveAddr};
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::vendor::ZephyrWriteBdAddr;
use nrf_sdc::{self as sdc, mpsl};
use sht4x::Sht4x;
use static_cell::StaticCell;
use tmag5273::{DeviceVariant, Tmag5273};
use {defmt_rtt as _, panic_probe as _};

mod board;
mod drivers;
mod tasks;

use drivers::led::Led;

const POFWARN_THRESHOLD: pac::regulators::vals::Threshold = pac::regulators::vals::Threshold::V28;

fn configure_power_fail_guard() {
    pac::REGULATORS.pofcon().write(|w| {
        w.set_pof(true);
        w.set_threshold(POFWARN_THRESHOLD);
        w.set_eventdisable(pac::regulators::vals::Eventdisable::ENABLED);
    });
    pac::RRAMC.power().config().modify(|w| {
        w.set_pof(pac::rramc::vals::Pof::ABORT);
    });
}

fn log_reset_reasons() {
    let reasons = pac::RESET.resetreas().read();
    DIAGNOSTICS_STATE.lock(|state| state.borrow_mut().note_resetreas(reasons.0));
    if reasons.0 == 0 {
        info!("[BOOT] resetreas=power_on");
        return;
    }

    info!(
        "[BOOT] resetreas raw=0x{:08x} pin={} dog0={} dog1={} ctrlapsoft={} ctrlaphard={} ctrlappin={} sreq={} lockup={} off={} lpcomp={} dif={} grtc={} nfc={} sectamper={}",
        reasons.0,
        reasons.resetpin(),
        reasons.dog0(),
        reasons.dog1(),
        reasons.ctrlapsoft(),
        reasons.ctrlaphard(),
        reasons.ctrlappin(),
        reasons.sreq(),
        reasons.lockup(),
        reasons.off(),
        reasons.lpcomp(),
        reasons.dif(),
        reasons.grtc(),
        reasons.nfc(),
        reasons.sectamper(),
    );
    pac::RESET.resetreas().write(|w| *w = reasons);
}

// --- Interrupt bindings ---

bind_interrupts!(struct Irqs {
    SERIAL20 => twim::InterruptHandler<peripherals::SERIAL20>;
    SAADC => saadc::InterruptHandler;
    SWI00 => mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => mpsl::ClockInterruptHandler;
    RADIO_0 => mpsl::HighPrioInterruptHandler;
    TIMER10 => mpsl::HighPrioInterruptHandler;
    GRTC_3 => mpsl::HighPrioInterruptHandler;
});

// --- Shared state ---

/// Signal from window_task → ble_task when state changes
pub static STATE_CHANGED: Signal<CriticalSectionRawMutex, tasks::window::WindowEvent> =
    Signal::new();

/// Signal from env_task → ble_task with environment data
pub static ENV_DATA: Signal<CriticalSectionRawMutex, window_sensor::telemetry::EnvReading> =
    Signal::new();

/// Signal from sensor tasks → setup_task with onboarding/runtime events.
pub static SETUP_EVENT: Signal<CriticalSectionRawMutex, window_sensor::setup::SetupEvent> =
    Signal::new();

/// Latest setup state for tasks that need context-sensitive event mapping.
pub static SETUP_STATE: Mutex<CriticalSectionRawMutex, RefCell<window_sensor::setup::SetupState>> =
    Mutex::new(RefCell::new(window_sensor::setup::SetupState::FactoryNew));

/// Signal from setup_task → future BLE/UX tasks with setup decisions.
pub static SETUP_CHANGED: Signal<CriticalSectionRawMutex, window_sensor::setup::SetupDecision> =
    Signal::new();

/// Signal from setup_task → window_task to capture the next calibration phase.
pub static WINDOW_CALIBRATION_REQUEST: Signal<
    CriticalSectionRawMutex,
    window_sensor::setup::CalibrationPhase,
> = Signal::new();

/// Signal from setup_task → blue LED task with the current setup hint.
pub static SETUP_LED_HINT: Signal<CriticalSectionRawMutex, window_sensor::setup::LedHint> =
    Signal::new();

/// Signal from MEMS task → future BLE task with BTHome button events.
pub static BUTTON_EVENT: Signal<CriticalSectionRawMutex, window_sensor::bthome::ButtonEvent> =
    Signal::new();

/// Sticky runtime diagnostics for issue reporting.
pub static DIAGNOSTICS_STATE: Mutex<
    CriticalSectionRawMutex,
    RefCell<window_sensor::diagnostics::DiagnosticsState>,
> = Mutex::new(RefCell::new(
    window_sensor::diagnostics::DiagnosticsState::new(),
));

// --- I2C bus type alias ---
pub type I2cBus = Twim<'static>;
pub type SharedI2cBus = Mutex<NoopRawMutex, RefCell<I2cBus>>;
pub type SettingsFlash = Mutex<NoopRawMutex, RefCell<nvmc::Nvmc<'static>>>;

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

fn bd_addr() -> bt_hci::param::BdAddr {
    let ficr = embassy_nrf::pac::FICR;
    let high = u64::from(ficr.deviceaddr(1).read());
    let addr = (high << 32) | u64::from(ficr.deviceaddr(0).read());
    let addr = addr | 0x0000_c000_0000_0000;
    bt_hci::param::BdAddr::new(unwrap!(addr.to_le_bytes()[..6].try_into()))
}

// --- Main entry ---

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.clock_speed = embassy_nrf::config::ClockSpeed::CK128;
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::ExternalXtal;

    let p = embassy_nrf::init(config);
    info!("[BOOT] window-sensor v{}", env!("CARGO_PKG_VERSION"));
    log_reset_reasons();
    configure_power_fail_guard();

    let mpsl_p = mpsl::Peripherals::new(
        p.GRTC_CH7,
        p.GRTC_CH8,
        p.GRTC_CH9,
        p.GRTC_CH10,
        p.GRTC_CH11,
        p.TIMER10,
        p.TIMER20,
        p.TEMP,
        p.PPI10_CH0,
        p.PPI20_CH1,
        p.PPIB11_CH0,
        p.PPIB21_CH0,
    );
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

    let sdc_p = sdc::Peripherals::new(
        p.PPI00_CH1,
        p.PPI00_CH3,
        p.PPI10_CH1,
        p.PPI10_CH2,
        p.PPI10_CH3,
        p.PPI10_CH4,
        p.PPI10_CH5,
        p.PPI10_CH6,
        p.PPI10_CH7,
        p.PPI10_CH8,
        p.PPI10_CH9,
        p.PPI10_CH10,
        p.PPI10_CH11,
        p.PPIB00_CH1,
        p.PPIB00_CH2,
        p.PPIB00_CH3,
        p.PPIB10_CH1,
        p.PPIB10_CH2,
        p.PPIB10_CH3,
    );
    static RNG: StaticCell<cracen::Cracen<'static, embassy_nrf::mode::Blocking>> =
        StaticCell::new();
    let rng = RNG.init(cracen::Cracen::new_blocking(p.CRACEN));
    static SDC_MEM: StaticCell<sdc::Mem<4096>> = StaticCell::new();
    static SDC: StaticCell<sdc::SoftdeviceController> = StaticCell::new();
    let sdc_builder = unwrap!(sdc::Builder::new());
    let sdc_builder = sdc_builder.support_adv().support_peripheral();
    let sdc_builder = unwrap!(sdc_builder.adv_count(1));
    let sdc_builder = unwrap!(sdc_builder.adv_buffer_cfg(31));
    let sdc = SDC.init(unwrap!(sdc_builder.build(
        sdc_p,
        rng,
        mpsl,
        SDC_MEM.init(sdc::Mem::new()),
    )));
    unwrap!(ZephyrWriteBdAddr::new(bd_addr()).exec(sdc).await);

    // --- LED boot flash (blue, 50ms) ---
    let mut led_blue = Led::new(Output::new(p.P1_08, Level::Low, OutputDrive::Standard));
    led_blue.flash_ms(50).await;

    // --- I2C bus init (100 kHz, shared between TMAG5273, LIS2DTW12, SHT4x) ---
    let mut twim_config = twim::Config::default();
    twim_config.frequency = twim::Frequency::K100;
    // External 16.9k pull-ups on SDA/SCL — no internal pull-up needed
    twim_config.sda_pullup = false;
    twim_config.scl_pullup = false;

    // TX RAM buffer for writes from flash (e.g., register address bytes)
    static TWIM_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let twim_buf = TWIM_BUF.init([0u8; 16]);
    let i2c = Twim::new(
        p.SERIAL20,
        Irqs,
        p.P1_03, // SDA (B4 on NORA-B216)
        p.P1_02, // SCL (A3 on NORA-B216)
        twim_config,
        twim_buf,
    );

    // Wrap I2C in a shared bus mutex so multiple async tasks can borrow it.
    static I2C_STATIC: StaticCell<SharedI2cBus> = StaticCell::new();
    let i2c = I2C_STATIC.init(Mutex::new(RefCell::new(i2c)));

    static SAADC_STATIC: StaticCell<AsyncMutex<NoopRawMutex, saadc::Saadc<'static, 1>>> =
        StaticCell::new();
    let vdd_saadc = SAADC_STATIC.init(AsyncMutex::new(tasks::environment::build_vdd_saadc(
        p.SAADC, Irqs,
    )));

    static SETTINGS_FLASH_STATIC: StaticCell<SettingsFlash> = StaticCell::new();
    let settings_flash =
        SETTINGS_FLASH_STATIC.init(Mutex::new(RefCell::new(nvmc::Nvmc::new(p.RRAMC))));
    let has_window_calibration = settings_flash.lock(|flash| {
        window_sensor::settings::load_soc_settings(&mut *flash.borrow_mut())
            .unwrap_or_default()
            .window_calibrated
    });

    // --- Probe sensors ---
    info!("[INIT] Probing I2C devices...");

    // TMAG5273 WHO_AM_I
    {
        let tmag_bus = I2cDevice::new(i2c);
        match Tmag5273::detect(tmag_bus, DeviceVariant::B1.default_address()) {
            Ok(_) => info!("[INIT] TMAG5273 OK (addr=0x22)"),
            Err(_) => error!("[INIT] TMAG5273 I2C error @ 0x22"),
        }
    }

    // LIS2DTW12 WHO_AM_I
    {
        let lis_bus = LisI2cInterface::new(I2cDevice::new(i2c), LisSlaveAddr::Alternative(true));
        let mut lis = Lis2dtw12::new(lis_bus);
        match lis.get_device_id() {
            Ok(id) => {
                if id == 0x44 {
                    info!("[INIT] LIS2DTW12 OK (id=0x{:02x})", id);
                } else {
                    error!(
                        "[INIT] LIS2DTW12 unexpected id=0x{:02x} (expected 0x44)",
                        id
                    );
                }
            }
            Err(_) => error!("[INIT] LIS2DTW12 I2C error @ 0x19"),
        }
    }

    // SHT4x — no WHO_AM_I register, try serial number read
    {
        let mut sht = Sht4x::<_, embassy_time::Delay>::new(I2cDevice::new(i2c));
        let mut delay = embassy_time::Delay;
        match sht.serial_number(&mut delay) {
            Ok(serial) => info!("[INIT] SHT4x OK (serial=0x{:08x})", serial),
            Err(_) => error!("[INIT] SHT4x I2C error @ 0x44"),
        }
    }

    // --- HALL interrupt pin (active-low from TMAG5273 INT) ---
    let hall_int = Input::new(p.P1_04, Pull::Up);

    // --- MEMS interrupt pins (active-low from LIS2DTW12 INT1/INT2) ---
    let mems_int1 = Input::new(p.P1_05, Pull::Up);
    let mems_int2 = Input::new(p.P1_06, Pull::Up);

    // --- Red LED for error/tamper indication (unused for now) ---
    let _led_red = Led::new(Output::new(p.P1_10, Level::Low, OutputDrive::Standard));

    // --- Spawn tasks ---
    info!("[INIT] Spawning tasks...");

    spawner.spawn(unwrap!(tasks::setup::setup_task(has_window_calibration)));
    spawner.spawn(unwrap!(tasks::setup::setup_led_task(led_blue)));
    spawner.spawn(unwrap!(tasks::window::window_task(i2c, hall_int, settings_flash)));
    spawner.spawn(unwrap!(tasks::mems::mems_task(
        i2c,
        mems_int1,
        mems_int2,
        window_sensor::mems_tuning::mems_config(),
    )));
    spawner.spawn(unwrap!(tasks::environment::environment_task(
        i2c,
        vdd_saadc,
        settings_flash,
        tasks::environment::EnvironmentConfig::default(),
    )));
    spawner.spawn(unwrap!(tasks::ble::ble_task(sdc)));

    info!("[INIT] System running — idle");

    loop {
        Timer::after_secs(60).await;
        trace!("[HEARTBEAT] alive");
    }
}
