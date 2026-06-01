//! OTA control and policy scaffold.

use crate::setup::SetupState;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OtaLayout {
    pub slot_size_bytes: u32,
    pub scratch_size_bytes: u32,
}

impl Default for OtaLayout {
    fn default() -> Self {
        Self {
            slot_size_bytes: 512 * 1024,
            scratch_size_bytes: 64 * 1024,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OtaPolicy {
    pub min_battery_pct: u8,
    pub max_image_size_bytes: u32,
    pub allow_debug_override: bool,
}

impl Default for OtaPolicy {
    fn default() -> Self {
        Self {
            min_battery_pct: 50,
            max_image_size_bytes: OtaLayout::default().slot_size_bytes,
            allow_debug_override: true,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OtaPrerequisites {
    pub battery_pct: u8,
    pub setup_state: SetupState,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OtaBlockReason {
    BatteryTooLow,
    SetupIncomplete,
    ImageTooLarge,
    SecondarySlotTooSmall,
    NotReady,
    VerificationFailed,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OtaSlot {
    Primary,
    Secondary,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BootOutcome {
    TrialBoot,
    Confirmed,
    Reverted,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OtaState {
    Idle,
    Advertised,
    Downloading,
    Verifying,
    ReadyToSwap,
    PendingReboot,
    AwaitingConfirmation,
    Completed,
    Blocked(OtaBlockReason),
    Failed(OtaBlockReason),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OtaCommand {
    Evaluate(OtaPrerequisites),
    StartTransfer { image_size_bytes: u32 },
    ChunkReceived { bytes: u32 },
    VerifyOk,
    VerifyFailed,
    RequestSwap,
    Booted(BootOutcome),
    ConfirmRunningImage,
    Abort,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OtaStatus {
    pub state: OtaState,
    pub target_slot: Option<OtaSlot>,
    pub bytes_received: u32,
    pub bytes_total: u32,
    pub advertise_ota: bool,
    pub accept_transfer: bool,
    pub reboot_to_swap: bool,
    pub confirm_running_image: bool,
}

impl OtaStatus {
    const fn new(
        state: OtaState,
        target_slot: Option<OtaSlot>,
        bytes_received: u32,
        bytes_total: u32,
    ) -> Self {
        let advertise_ota = matches!(state, OtaState::Advertised);
        let accept_transfer = matches!(state, OtaState::Downloading);
        let reboot_to_swap = matches!(state, OtaState::PendingReboot);
        let confirm_running_image = matches!(state, OtaState::AwaitingConfirmation);

        Self {
            state,
            target_slot,
            bytes_received,
            bytes_total,
            advertise_ota,
            accept_transfer,
            reboot_to_swap,
            confirm_running_image,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OtaController {
    policy: OtaPolicy,
    layout: OtaLayout,
    state: OtaState,
    target_slot: Option<OtaSlot>,
    bytes_received: u32,
    bytes_total: u32,
}

impl OtaController {
    pub const fn new(policy: OtaPolicy, layout: OtaLayout) -> Self {
        Self {
            policy,
            layout,
            state: OtaState::Idle,
            target_slot: None,
            bytes_received: 0,
            bytes_total: 0,
        }
    }

    pub const fn state(&self) -> OtaState {
        self.state
    }

    pub fn apply(&mut self, command: OtaCommand) -> OtaStatus {
        match command {
            OtaCommand::Evaluate(prereq) => {
                self.state = match self.block_reason(prereq) {
                    Some(reason) => OtaState::Blocked(reason),
                    None => OtaState::Advertised,
                };
            }
            OtaCommand::StartTransfer { image_size_bytes } => {
                if !matches!(self.state, OtaState::Advertised) {
                    self.state = OtaState::Blocked(OtaBlockReason::NotReady);
                } else if image_size_bytes > self.policy.max_image_size_bytes {
                    self.state = OtaState::Blocked(OtaBlockReason::ImageTooLarge);
                } else if image_size_bytes > self.layout.slot_size_bytes {
                    self.state = OtaState::Blocked(OtaBlockReason::SecondarySlotTooSmall);
                } else {
                    self.state = OtaState::Downloading;
                    self.target_slot = Some(OtaSlot::Secondary);
                    self.bytes_total = image_size_bytes;
                    self.bytes_received = 0;
                }
            }
            OtaCommand::ChunkReceived { bytes } => {
                if matches!(self.state, OtaState::Downloading) {
                    self.bytes_received = self.bytes_received.saturating_add(bytes);
                    if self.bytes_received >= self.bytes_total && self.bytes_total != 0 {
                        self.bytes_received = self.bytes_total;
                        self.state = OtaState::Verifying;
                    }
                }
            }
            OtaCommand::VerifyOk => {
                if matches!(self.state, OtaState::Verifying) {
                    self.state = OtaState::ReadyToSwap;
                }
            }
            OtaCommand::VerifyFailed => {
                self.state = OtaState::Failed(OtaBlockReason::VerificationFailed);
            }
            OtaCommand::RequestSwap => {
                if matches!(self.state, OtaState::ReadyToSwap) {
                    self.state = OtaState::PendingReboot;
                }
            }
            OtaCommand::Booted(outcome) => {
                self.state = match outcome {
                    BootOutcome::TrialBoot => OtaState::AwaitingConfirmation,
                    BootOutcome::Confirmed => OtaState::Completed,
                    BootOutcome::Reverted => OtaState::Failed(OtaBlockReason::VerificationFailed),
                };
            }
            OtaCommand::ConfirmRunningImage => {
                if matches!(self.state, OtaState::AwaitingConfirmation) {
                    self.state = OtaState::Completed;
                    self.target_slot = Some(OtaSlot::Primary);
                }
            }
            OtaCommand::Abort => {
                self.state = OtaState::Idle;
                self.target_slot = None;
                self.bytes_received = 0;
                self.bytes_total = 0;
            }
        }

        OtaStatus::new(
            self.state,
            self.target_slot,
            self.bytes_received,
            self.bytes_total,
        )
    }

    fn block_reason(&self, prereq: OtaPrerequisites) -> Option<OtaBlockReason> {
        if prereq.battery_pct < self.policy.min_battery_pct {
            return Some(OtaBlockReason::BatteryTooLow);
        }

        if prereq.setup_state != SetupState::Ready
            && !(self.policy.allow_debug_override && prereq.setup_state == SetupState::Debug)
        {
            return Some(OtaBlockReason::SetupIncomplete);
        }

        None
    }
}

impl Default for OtaController {
    fn default() -> Self {
        Self::new(OtaPolicy::default(), OtaLayout::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ready_device_with_good_battery_advertises_ota() {
        let mut ota = OtaController::default();

        let status = ota.apply(OtaCommand::Evaluate(OtaPrerequisites {
            battery_pct: 82,
            setup_state: SetupState::Ready,
        }));

        assert_eq!(status.state, OtaState::Advertised);
        assert!(status.advertise_ota);
        assert_eq!(status.target_slot, None);
    }

    #[test]
    fn low_battery_blocks_ota() {
        let mut ota = OtaController::default();

        let status = ota.apply(OtaCommand::Evaluate(OtaPrerequisites {
            battery_pct: 20,
            setup_state: SetupState::Ready,
        }));

        assert_eq!(
            status.state,
            OtaState::Blocked(OtaBlockReason::BatteryTooLow)
        );
        assert!(!status.advertise_ota);
    }

    #[test]
    fn incomplete_setup_blocks_ota() {
        let mut ota = OtaController::default();

        let status = ota.apply(OtaCommand::Evaluate(OtaPrerequisites {
            battery_pct: 82,
            setup_state: SetupState::Discovery,
        }));

        assert_eq!(
            status.state,
            OtaState::Blocked(OtaBlockReason::SetupIncomplete)
        );
    }

    #[test]
    fn debug_mode_can_override_setup_gate() {
        let mut ota = OtaController::default();

        let status = ota.apply(OtaCommand::Evaluate(OtaPrerequisites {
            battery_pct: 82,
            setup_state: SetupState::Debug,
        }));

        assert_eq!(status.state, OtaState::Advertised);
    }

    #[test]
    fn oversized_image_is_rejected() {
        let mut ota = OtaController::default();
        let _ = ota.apply(OtaCommand::Evaluate(OtaPrerequisites {
            battery_pct: 82,
            setup_state: SetupState::Ready,
        }));

        let status = ota.apply(OtaCommand::StartTransfer {
            image_size_bytes: 1024 * 1024,
        });

        assert_eq!(
            status.state,
            OtaState::Blocked(OtaBlockReason::ImageTooLarge)
        );
    }

    #[test]
    fn image_larger_than_secondary_slot_is_rejected() {
        let policy = OtaPolicy {
            max_image_size_bytes: 700 * 1024,
            ..OtaPolicy::default()
        };
        let layout = OtaLayout {
            slot_size_bytes: 512 * 1024,
            scratch_size_bytes: 64 * 1024,
        };
        let mut ota = OtaController::new(policy, layout);
        let _ = ota.apply(OtaCommand::Evaluate(OtaPrerequisites {
            battery_pct: 82,
            setup_state: SetupState::Ready,
        }));

        let status = ota.apply(OtaCommand::StartTransfer {
            image_size_bytes: 600 * 1024,
        });

        assert_eq!(
            status.state,
            OtaState::Blocked(OtaBlockReason::SecondarySlotTooSmall)
        );
    }

    #[test]
    fn ota_progresses_from_download_to_confirmed_boot() {
        let mut ota = OtaController::default();
        let _ = ota.apply(OtaCommand::Evaluate(OtaPrerequisites {
            battery_pct: 82,
            setup_state: SetupState::Ready,
        }));

        let downloading = ota.apply(OtaCommand::StartTransfer {
            image_size_bytes: 1024,
        });
        assert_eq!(downloading.state, OtaState::Downloading);
        assert!(downloading.accept_transfer);
        assert_eq!(downloading.target_slot, Some(OtaSlot::Secondary));

        let verifying = ota.apply(OtaCommand::ChunkReceived { bytes: 1024 });
        assert_eq!(verifying.state, OtaState::Verifying);
        assert_eq!(verifying.bytes_received, 1024);

        let ready = ota.apply(OtaCommand::VerifyOk);
        assert_eq!(ready.state, OtaState::ReadyToSwap);

        let rebooting = ota.apply(OtaCommand::RequestSwap);
        assert_eq!(rebooting.state, OtaState::PendingReboot);
        assert!(rebooting.reboot_to_swap);

        let trial_boot = ota.apply(OtaCommand::Booted(BootOutcome::TrialBoot));
        assert_eq!(trial_boot.state, OtaState::AwaitingConfirmation);
        assert!(trial_boot.confirm_running_image);

        let done = ota.apply(OtaCommand::ConfirmRunningImage);
        assert_eq!(done.state, OtaState::Completed);
        assert_eq!(done.target_slot, Some(OtaSlot::Primary));
    }

    #[test]
    fn verification_failure_moves_to_failed_state() {
        let mut ota = OtaController::default();
        let _ = ota.apply(OtaCommand::Evaluate(OtaPrerequisites {
            battery_pct: 82,
            setup_state: SetupState::Ready,
        }));
        let _ = ota.apply(OtaCommand::StartTransfer {
            image_size_bytes: 512,
        });
        let _ = ota.apply(OtaCommand::ChunkReceived { bytes: 512 });

        let failed = ota.apply(OtaCommand::VerifyFailed);
        assert_eq!(
            failed.state,
            OtaState::Failed(OtaBlockReason::VerificationFailed)
        );
    }

    #[test]
    fn reverted_trial_boot_is_reported_as_failed() {
        let mut ota = OtaController::default();

        let failed = ota.apply(OtaCommand::Booted(BootOutcome::Reverted));
        assert_eq!(
            failed.state,
            OtaState::Failed(OtaBlockReason::VerificationFailed)
        );
    }
}
