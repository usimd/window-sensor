//! BLE task — BTHome v2 advertising via nrf-sdc.
//!
//! Runtime BLE stays advertisement-only. Setup and calibration are driven by the
//! local MEMS and magnet UX, while OTA remains a separate future path.

use bt_hci::cmd::SyncCmd;
use bt_hci::cmd::le::{LeSetAdvData, LeSetAdvEnable, LeSetAdvParams};
use bt_hci::param::{AddrKind, AdvChannelMap, AdvFilterPolicy, AdvKind, BdAddr, Duration};
use defmt::*;
use embassy_futures::select::{Either4, select4};
use embassy_time::{Duration as EmbassyDuration, Timer};
use nrf_sdc::SoftdeviceController;

use crate::{BUTTON_EVENT, DIAGNOSTICS_STATE, ENV_DATA, SETUP_CHANGED, SETUP_STATE, STATE_CHANGED};

pub type EnvReading = window_sensor::telemetry::EnvReading;
pub type PacketCounter = window_sensor::telemetry::PacketCounter;
pub type ButtonEvent = window_sensor::bthome::ButtonEvent;
pub type BootOutcome = window_sensor::ota::BootOutcome;
pub type OtaLayout = window_sensor::ota::OtaLayout;
pub type OtaPolicy = window_sensor::ota::OtaPolicy;
pub type OtaSlot = window_sensor::ota::OtaSlot;
pub type OtaState = window_sensor::ota::OtaState;
pub type OtaStatus = window_sensor::ota::OtaStatus;
pub type SetupDecision = window_sensor::setup::SetupDecision;
pub type SetupState = window_sensor::setup::SetupState;
pub type WindowStatus = window_sensor::telemetry::WindowStatus;

pub const AD_TYPE_SERVICE_DATA_UUID16: u8 = window_sensor::telemetry::AD_TYPE_SERVICE_DATA_UUID16;

const ACTIVE_ADV_INTERVAL_MS: u32 = 20;
const ACTIVE_ADV_WINDOW: EmbassyDuration = EmbassyDuration::from_millis(60);
const RETRY_DELAY: EmbassyDuration = EmbassyDuration::from_secs(5);
#[embassy_executor::task]
pub async fn ble_task(controller: &'static SoftdeviceController<'static>) {
    info!("[BLE] Task started");

    let mut packet_counter = PacketCounter::new();
    let mut status = WindowStatus {
        state: window_sensor::classifier::WindowState::Closed,
        tampered: false,
        problem: current_problem_state(),
    };
    let mut setup_state = current_setup_state();

    loop {
        match select4(
            STATE_CHANGED.wait(),
            ENV_DATA.wait(),
            BUTTON_EVENT.wait(),
            SETUP_CHANGED.wait(),
        )
        .await
        {
            Either4::First(event) => {
                trace!(
                    "[BLE] state={} peak={}x0.1mT",
                    state_str(event.state),
                    event.peak_mt_x10
                );

                status.state = event.state;
                status.problem = current_problem_state();
                if status.state == window_sensor::classifier::WindowState::Closed {
                    status.tampered = false;
                }

                if send_state_burst(controller, &mut packet_counter, status)
                    .await
                    .is_err()
                {
                    warn!("[BLE] State burst failed");
                    Timer::after(RETRY_DELAY).await;
                }
            }
            Either4::Second(env) => {
                status.problem = current_problem_state();
                if send_heartbeat(controller, &mut packet_counter, env, status)
                    .await
                    .is_err()
                {
                    warn!("[BLE] Heartbeat failed");
                    Timer::after(RETRY_DELAY).await;
                }
            }
            Either4::Third(event) => {
                if send_button_event(controller, &mut packet_counter, event)
                    .await
                    .is_err()
                {
                    warn!("[BLE] Button event failed");
                    Timer::after(RETRY_DELAY).await;
                }
            }
            Either4::Fourth(decision) => {
                let previous = setup_state;
                setup_state = decision.state;
                status.problem = current_problem_state();

                if setup_state == SetupState::NeedsRecalibration {
                    status.tampered = true;
                    if send_state_burst(controller, &mut packet_counter, status)
                        .await
                        .is_err()
                    {
                        warn!("[BLE] Tamper burst failed");
                        Timer::after(RETRY_DELAY).await;
                    }
                    continue;
                }

                if previous == SetupState::NeedsRecalibration
                    && setup_state != SetupState::NeedsRecalibration
                    && status.state == window_sensor::classifier::WindowState::Closed
                {
                    status.tampered = false;
                }

                if previous != setup_state
                    && send_state_once(controller, &mut packet_counter, status)
                        .await
                        .is_err()
                {
                    warn!("[BLE] Setup state update failed");
                    Timer::after(RETRY_DELAY).await;
                }
            }
        }
    }
}

pub fn encode_state_advertisement(buf: &mut [u8], packet_id: u8, status: WindowStatus) -> usize {
    window_sensor::telemetry::encode_state_advertisement(buf, packet_id, status)
}

pub fn encode_heartbeat_advertisement(
    buf: &mut [u8],
    packet_id: u8,
    env: EnvReading,
    status: WindowStatus,
) -> usize {
    window_sensor::telemetry::encode_heartbeat_advertisement(buf, packet_id, env, status)
}

pub fn encode_button_advertisement(buf: &mut [u8], packet_id: u8, event: ButtonEvent) -> usize {
    window_sensor::telemetry::encode_button_advertisement(buf, packet_id, event)
}

pub fn should_prioritize_calibration(setup: SetupState) -> bool {
    setup.calibration_required()
}

pub fn should_advertise_ota(status: OtaStatus) -> bool {
    status.advertise_ota
}

pub fn should_confirm_running_image(status: OtaStatus) -> bool {
    status.confirm_running_image
}

async fn send_state_burst(
    controller: &SoftdeviceController<'static>,
    packet_counter: &mut PacketCounter,
    status: WindowStatus,
) -> Result<(), bt_hci::param::Error> {
    let packet_id = packet_counter.next_packet_id();
    let mut adv = [0u8; 31];
    let len = encode_state_advertisement(&mut adv, packet_id, status);

    for repeat in 0..window_sensor::advertising::STATE_BURST_COUNT {
        transmit_advertisement(controller, &adv[..len]).await?;
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
    controller: &SoftdeviceController<'static>,
    packet_counter: &mut PacketCounter,
    status: WindowStatus,
) -> Result<(), bt_hci::param::Error> {
    let packet_id = packet_counter.next_packet_id();
    let mut adv = [0u8; 31];
    let len = encode_state_advertisement(&mut adv, packet_id, status);
    transmit_advertisement(controller, &adv[..len]).await
}

async fn send_heartbeat(
    controller: &SoftdeviceController<'static>,
    packet_counter: &mut PacketCounter,
    env: EnvReading,
    status: WindowStatus,
) -> Result<(), bt_hci::param::Error> {
    let packet_id = packet_counter.next_packet_id();
    let mut adv = [0u8; 31];
    let len = encode_heartbeat_advertisement(&mut adv, packet_id, env, status);
    transmit_advertisement(controller, &adv[..len]).await
}

async fn send_button_event(
    controller: &SoftdeviceController<'static>,
    packet_counter: &mut PacketCounter,
    event: ButtonEvent,
) -> Result<(), bt_hci::param::Error> {
    let packet_id = packet_counter.next_packet_id();
    let mut adv = [0u8; 31];
    let len = encode_button_advertisement(&mut adv, packet_id, event);
    transmit_advertisement(controller, &adv[..len]).await
}

async fn transmit_advertisement(
    controller: &SoftdeviceController<'static>,
    payload: &[u8],
) -> Result<(), bt_hci::param::Error> {
    let mut adv_data = [0u8; 31];
    adv_data[..payload.len()].copy_from_slice(payload);

    let _ = LeSetAdvEnable::new(false).exec(controller).await;
    LeSetAdvParams::new(
        Duration::from_millis(ACTIVE_ADV_INTERVAL_MS),
        Duration::from_millis(ACTIVE_ADV_INTERVAL_MS),
        AdvKind::AdvNonconnInd,
        AddrKind::PUBLIC,
        AddrKind::PUBLIC,
        BdAddr::default(),
        AdvChannelMap::ALL,
        AdvFilterPolicy::default(),
    )
    .exec(controller)
    .await
    .map_err(into_hci_error)?;
    LeSetAdvData::new(payload.len() as u8, adv_data)
        .exec(controller)
        .await
        .map_err(into_hci_error)?;
    LeSetAdvEnable::new(true)
        .exec(controller)
        .await
        .map_err(into_hci_error)?;

    Timer::after(ACTIVE_ADV_WINDOW).await;

    LeSetAdvEnable::new(false)
        .exec(controller)
        .await
        .map_err(into_hci_error)
}

fn current_setup_state() -> SetupState {
    SETUP_STATE.lock(|state| *state.borrow())
}

fn current_problem_state() -> bool {
    DIAGNOSTICS_STATE.lock(|state| state.borrow().problem_active())
}

fn state_str(state: window_sensor::classifier::WindowState) -> &'static str {
    match state {
        window_sensor::classifier::WindowState::Closed => "CLOSED",
        window_sensor::classifier::WindowState::Tilt => "TILT",
        window_sensor::classifier::WindowState::Open => "OPEN",
    }
}

fn into_hci_error(error: bt_hci::cmd::Error<nrf_sdc::Error>) -> bt_hci::param::Error {
    match error {
        bt_hci::cmd::Error::Hci(error) => error,
        bt_hci::cmd::Error::Io(_) => bt_hci::param::Error::HARDWARE_FAILURE,
    }
}
