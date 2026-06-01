//! OTA bootloader-facing contract, kept host-testable until MCUboot hooks land.

use crate::ota::{BootOutcome, OtaBlockReason, OtaLayout, OtaState, OtaStatus};
use crate::partition::PartitionLayout;

impl PartitionLayout {
    pub const fn ota_layout(&self) -> OtaLayout {
        OtaLayout {
            slot_size_bytes: self.secondary_slot_size,
            scratch_size_bytes: self.scratch_size,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BootAction {
    None,
    MarkPendingSwap,
    ConfirmRunningImage,
    RecordFailure,
}

pub fn action_for_status(status: OtaStatus) -> BootAction {
    match status.state {
        OtaState::PendingReboot => BootAction::MarkPendingSwap,
        OtaState::AwaitingConfirmation if status.confirm_running_image => {
            BootAction::ConfirmRunningImage
        }
        OtaState::Failed(_) => BootAction::RecordFailure,
        _ => BootAction::None,
    }
}

pub fn status_from_boot_outcome(outcome: BootOutcome) -> OtaState {
    match outcome {
        BootOutcome::TrialBoot => OtaState::AwaitingConfirmation,
        BootOutcome::Confirmed => OtaState::Completed,
        BootOutcome::Reverted => OtaState::Failed(OtaBlockReason::VerificationFailed),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ota::OtaSlot;
    use crate::partition::LayoutError;

    #[test]
    fn default_layout_is_valid() {
        let layout = PartitionLayout::mcuboot_default();

        assert_eq!(layout.validate(), Ok(()));
        assert_eq!(layout.app_link_origin(), 0x0001_0000);
        assert_eq!(layout.app_link_length(), 384 * 1024);
        assert_eq!(layout.ota_layout().slot_size_bytes, 384 * 1024);
    }

    #[test]
    fn layout_rejects_slot_size_mismatch() {
        let mut layout = PartitionLayout::mcuboot_default();
        layout.secondary_slot_size = 320 * 1024;

        assert_eq!(layout.validate(), Err(LayoutError::SlotSizeMismatch));
    }

    #[test]
    fn layout_rejects_invalid_region_order() {
        let mut layout = PartitionLayout::mcuboot_default();
        layout.secondary_slot_start = layout.primary_slot_start - 4;

        assert_eq!(layout.validate(), Err(LayoutError::RegionOverlap));
    }

    #[test]
    fn pending_reboot_requests_mark_swap() {
        let status = OtaStatus {
            state: OtaState::PendingReboot,
            target_slot: Some(OtaSlot::Secondary),
            bytes_received: 100,
            bytes_total: 100,
            advertise_ota: false,
            accept_transfer: false,
            reboot_to_swap: true,
            confirm_running_image: false,
        };

        assert_eq!(action_for_status(status), BootAction::MarkPendingSwap);
    }

    #[test]
    fn awaiting_confirmation_requests_image_confirm() {
        let status = OtaStatus {
            state: OtaState::AwaitingConfirmation,
            target_slot: Some(OtaSlot::Primary),
            bytes_received: 100,
            bytes_total: 100,
            advertise_ota: false,
            accept_transfer: false,
            reboot_to_swap: false,
            confirm_running_image: true,
        };

        assert_eq!(action_for_status(status), BootAction::ConfirmRunningImage);
    }
}
