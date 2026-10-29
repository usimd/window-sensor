//! BLE task: BTHome v2 advertising plus a periodic connectable update window.
//!
//! Sensor reporting stays advertisement-only. Every `ota_window::WINDOW_PERIOD_S`, when battery and
//! setup allow it, the device advertises connectably for `WINDOW_DURATION_MS` and serves MCUmgr SMP
//! to an OTA client (Home Assistant) for at most `MAX_CONNECTION_S`.

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, Either4, select, select4};
use embassy_nrf::wdt::WatchdogHandle;
use embassy_time::{Duration, Instant, Timer};
use nrf_sdc::SoftdeviceController;
use static_cell::StaticCell;
use trouble_host::prelude::*;
use window_sensor::bthome::ButtonEvent;
use window_sensor::classifier::WindowState;
use window_sensor::ota::{OtaCommand, OtaController, OtaPrerequisites};
use window_sensor::ota_window::{self, WindowSchedule};
use window_sensor::partition::MCU_BOOT_LAYOUT;
use window_sensor::setup::SetupState;
use window_sensor::telemetry::{self, EnvReading, PacketCounter, WindowStatus};

use super::ota::{OtaServer, SharedFlash, SmpServer};
use crate::{
    BUTTON_EVENT, DIAGNOSTICS_STATE, ENV_DATA, SETUP_CHANGED, SETUP_STATE, STATE_CHANGED,
    SettingsFlash,
};

type Sdc = SoftdeviceController<'static>;
type BleError = BleHostError<nrf_sdc::Error>;
type BlePeripheral = Peripheral<'static, Sdc, DefaultPacketPool>;

/// One peripheral link; L2CAP channels for signalling and ATT.
const CONNECTIONS: usize = 1;
const L2CAP_CHANNELS: usize = 2;

const BURST_ADV_INTERVAL: Duration = Duration::from_millis(20);
const BURST_ADV_WINDOW: Duration = Duration::from_millis(60);
const RETRY_DELAY: Duration = Duration::from_secs(5);
const RESET_DELAY: Duration = Duration::from_millis(200);

/// The BLE loop wakes at least once per update window; two missed windows mean it is stuck.
pub const WATCHDOG_TIMEOUT_S: u32 = 2 * ota_window::WINDOW_PERIOD_S;
/// A test-swapped image that cannot confirm itself within this time is reverted by MCUboot.
const TRIAL_GRACE: Duration = Duration::from_secs(15 * 60);

/// Static random address derived from FICR.DEVICEADDR, little-endian.
pub fn device_address() -> [u8; 6] {
    let ficr = embassy_nrf::pac::FICR;
    let high = u64::from(ficr.deviceaddr(1).read());
    let addr = (high << 32) | u64::from(ficr.deviceaddr(0).read()) | 0x0000_c000_0000_0000;
    let mut out = [0u8; 6];
    out.copy_from_slice(&addr.to_le_bytes()[..6]);
    out
}

pub fn start(
    spawner: Spawner,
    sdc: Sdc,
    settings_flash: &'static SettingsFlash,
    watchdog: WatchdogHandle,
) {
    static RESOURCES: StaticCell<HostResources<DefaultPacketPool, CONNECTIONS, L2CAP_CHANNELS>> =
        StaticCell::new();
    static STACK: StaticCell<Stack<'static, Sdc, DefaultPacketPool>> = StaticCell::new();
    let stack: &'static Stack<'static, Sdc, DefaultPacketPool> = STACK.init(
        trouble_host::new(sdc, RESOURCES.init(HostResources::new()))
            .set_random_address(Address::random(device_address()))
            .build(),
    );
    spawner.spawn(unwrap!(ble_runner_task(stack.runner())));
    spawner.spawn(unwrap!(ble_task(
        stack.peripheral(),
        settings_flash,
        watchdog
    )));
}

#[embassy_executor::task]
async fn ble_runner_task(mut runner: Runner<'static, Sdc, DefaultPacketPool>) {
    loop {
        if let Err(error) = runner.run().await {
            error!("[BLE] host stopped: {}", Debug2Format(&error));
            Timer::after(RETRY_DELAY).await;
        }
    }
}

#[embassy_executor::task]
async fn ble_task(
    mut peripheral: BlePeripheral,
    settings_flash: &'static SettingsFlash,
    mut watchdog: WatchdogHandle,
) {
    info!("[BLE] Task started");

    static NAME: StaticCell<[u8; ota_window::NAME_LEN]> = StaticCell::new();
    let name = NAME.init(ota_window::local_name(device_address()));
    let server = unwrap!(OtaServer::new_with_config(GapConfig::Peripheral(
        PeripheralConfig {
            name: unwrap!(core::str::from_utf8(name)),
            appearance: &appearance::sensor::GENERIC_SENSOR,
        }
    )));
    let mut smp = SmpServer::new(SharedFlash(settings_flash), &MCU_BOOT_LAYOUT);
    let mut ota = OtaController::default();
    let mut schedule = WindowSchedule::new(Instant::now().as_millis());
    let mut image_confirmed = smp.running_image_confirmed();
    let trial_deadline = Instant::now() + TRIAL_GRACE;
    if !image_confirmed {
        info!("[OTA] trial boot, confirming after first heartbeat");
    }
    let mut last_env: Option<EnvReading> = None;
    let mut window_seq: u8 = 0;

    let mut packet_counter = PacketCounter::new();
    let mut status = WindowStatus {
        state: WindowState::Closed,
        tampered: false,
        problem: current_problem_state(),
    };
    let mut setup_state = current_setup_state();

    loop {
        watchdog.pet();
        let mut wake_at = Instant::from_millis(schedule.next_at_ms());
        if !image_confirmed {
            wake_at = wake_at.min(trial_deadline);
        }
        let events = select4(
            STATE_CHANGED.wait(),
            ENV_DATA.wait(),
            BUTTON_EVENT.wait(),
            SETUP_CHANGED.wait(),
        );
        let window = Timer::at(wake_at);
        match select(events, window).await {
            Either::First(Either4::First(event)) => {
                trace!(
                    "[BLE] state={} peak={}x0.1mT",
                    state_str(event.state),
                    event.peak_mt_x10
                );
                status.state = event.state;
                status.problem = current_problem_state();
                if status.state == WindowState::Closed {
                    status.tampered = false;
                }
                if send_state_burst(&mut peripheral, &mut packet_counter, status)
                    .await
                    .is_err()
                {
                    warn!("[BLE] State burst failed");
                    Timer::after(RETRY_DELAY).await;
                }
            }
            Either::First(Either4::Second(env)) => {
                last_env = Some(env);
                status.problem = current_problem_state();
                if send_heartbeat(&mut peripheral, &mut packet_counter, env, status)
                    .await
                    .is_err()
                {
                    warn!("[BLE] Heartbeat failed");
                    Timer::after(RETRY_DELAY).await;
                } else if !image_confirmed {
                    // Sensors read and radio works: keep this image across the next reset.
                    image_confirmed = smp.confirm_running_image();
                    info!("[OTA] running image confirmed={}", image_confirmed);
                }
            }
            Either::First(Either4::Third(event)) => {
                if send_button_event(&mut peripheral, &mut packet_counter, event)
                    .await
                    .is_err()
                {
                    warn!("[BLE] Button event failed");
                    Timer::after(RETRY_DELAY).await;
                }
            }
            Either::First(Either4::Fourth(decision)) => {
                let previous = setup_state;
                setup_state = decision.state;
                status.problem = current_problem_state();

                if setup_state == SetupState::NeedsRecalibration {
                    status.tampered = true;
                    if send_state_burst(&mut peripheral, &mut packet_counter, status)
                        .await
                        .is_err()
                    {
                        warn!("[BLE] Tamper burst failed");
                        Timer::after(RETRY_DELAY).await;
                    }
                    continue;
                }

                if previous == SetupState::NeedsRecalibration && status.state == WindowState::Closed
                {
                    status.tampered = false;
                }

                if previous != setup_state
                    && send_state_once(&mut peripheral, &mut packet_counter, status)
                        .await
                        .is_err()
                {
                    warn!("[BLE] Setup state update failed");
                    Timer::after(RETRY_DELAY).await;
                }
            }
            Either::Second(()) => {
                let now = Instant::now();
                if !image_confirmed && now >= trial_deadline {
                    warn!("[OTA] trial image unconfirmed, resetting to revert");
                    Timer::after(RESET_DELAY).await;
                    cortex_m::peripheral::SCB::sys_reset();
                }
                if !schedule.due(now.as_millis()) {
                    continue;
                }
                schedule.advance(now.as_millis());
                let prerequisites = OtaPrerequisites {
                    battery_pct: last_env.map_or(0, |env| env.battery_soc_pct),
                    setup_state,
                };
                if !ota.apply(OtaCommand::Evaluate(prerequisites)).advertise_ota {
                    debug!("[OTA] window skipped: {}", Debug2Format(&ota.state()));
                    continue;
                }
                window_seq = window_seq.wrapping_add(1);
                let mut ad = [0u8; 31];
                let len = telemetry::encode_state_advertisement(
                    &mut ad,
                    packet_counter.next_packet_id(),
                    status,
                );
                if let Err(error) =
                    run_update_window(&mut peripheral, &server, &mut smp, &ad[..len], window_seq)
                        .await
                {
                    warn!("[OTA] window failed: {}", Debug2Format(&error));
                }
            }
        }
    }
}

/// Advertises connectably for one window and serves a single SMP client; resets if it asks to.
async fn run_update_window(
    peripheral: &mut BlePeripheral,
    server: &OtaServer<'_>,
    smp: &mut SmpServer,
    service_data: &[u8],
    window_seq: u8,
) -> Result<(), BleError> {
    let mut adv = [0u8; 31];
    let adv_len = ota_window::encode_connectable_advertisement(&mut adv, service_data, window_seq);
    let mut scan = [0u8; 31];
    let scan_len = ota_window::encode_scan_response(&mut scan, device_address());

    let advertiser = peripheral
        .advertise(
            &adv_params(Duration::from_millis(u64::from(
                ota_window::WINDOW_ADV_INTERVAL_MS,
            ))),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv[..adv_len],
                scan_data: &scan[..scan_len],
            },
        )
        .await?;
    let window = Timer::after_millis(u64::from(ota_window::WINDOW_DURATION_MS));
    let conn = match select(advertiser.accept(), window).await {
        Either::First(conn) => conn?,
        Either::Second(()) => return Ok(()),
    };
    let conn = conn.with_attribute_server(server)?;
    info!("[OTA] client connected");

    let limit = Timer::after_secs(u64::from(ota_window::MAX_CONNECTION_S));
    let reset = match select(super::ota::serve(server, &conn, smp), limit).await {
        Either::First(reset) => reset,
        Either::Second(()) => {
            warn!("[OTA] connection time limit reached");
            false
        }
    };
    if reset {
        info!("[OTA] reset requested");
        Timer::after(RESET_DELAY).await;
        cortex_m::peripheral::SCB::sys_reset();
    }
    Ok(())
}

fn adv_params(interval: Duration) -> AdvertisementParameters {
    AdvertisementParameters {
        interval_min: interval,
        interval_max: interval,
        ..Default::default()
    }
}

async fn send_state_burst(
    peripheral: &mut BlePeripheral,
    packet_counter: &mut PacketCounter,
    status: WindowStatus,
) -> Result<(), BleError> {
    let mut adv = [0u8; 31];
    let len =
        telemetry::encode_state_advertisement(&mut adv, packet_counter.next_packet_id(), status);

    for repeat in 0..window_sensor::advertising::STATE_BURST_COUNT {
        transmit_advertisement(peripheral, &adv[..len]).await?;
        if repeat + 1 < window_sensor::advertising::STATE_BURST_COUNT {
            Timer::after_millis(u64::from(
                window_sensor::advertising::STATE_BURST_INTERVAL_MS,
            ))
            .await;
        }
    }
    Ok(())
}

async fn send_state_once(
    peripheral: &mut BlePeripheral,
    packet_counter: &mut PacketCounter,
    status: WindowStatus,
) -> Result<(), BleError> {
    let mut adv = [0u8; 31];
    let len =
        telemetry::encode_state_advertisement(&mut adv, packet_counter.next_packet_id(), status);
    transmit_advertisement(peripheral, &adv[..len]).await
}

async fn send_heartbeat(
    peripheral: &mut BlePeripheral,
    packet_counter: &mut PacketCounter,
    env: EnvReading,
    status: WindowStatus,
) -> Result<(), BleError> {
    let mut adv = [0u8; 31];
    let len = telemetry::encode_heartbeat_advertisement(
        &mut adv,
        packet_counter.next_packet_id(),
        env,
        status,
    );
    transmit_advertisement(peripheral, &adv[..len]).await
}

async fn send_button_event(
    peripheral: &mut BlePeripheral,
    packet_counter: &mut PacketCounter,
    event: ButtonEvent,
) -> Result<(), BleError> {
    let mut adv = [0u8; 31];
    let len =
        telemetry::encode_button_advertisement(&mut adv, packet_counter.next_packet_id(), event);
    transmit_advertisement(peripheral, &adv[..len]).await
}

/// Non-connectable burst; dropping the advertiser stops advertising.
async fn transmit_advertisement(
    peripheral: &mut BlePeripheral,
    payload: &[u8],
) -> Result<(), BleError> {
    let _advertiser = peripheral
        .advertise(
            &adv_params(BURST_ADV_INTERVAL),
            Advertisement::NonconnectableNonscannableUndirected { adv_data: payload },
        )
        .await?;
    Timer::after(BURST_ADV_WINDOW).await;
    Ok(())
}

fn current_setup_state() -> SetupState {
    SETUP_STATE.lock(|state| *state.borrow())
}

fn current_problem_state() -> bool {
    DIAGNOSTICS_STATE.lock(|state| state.borrow().problem_active())
}

fn state_str(state: WindowState) -> &'static str {
    match state {
        WindowState::Closed => "CLOSED",
        WindowState::Tilt => "TILT",
        WindowState::Open => "OPEN",
    }
}
