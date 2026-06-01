#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PartitionLayout {
    pub bootloader_start: u32,
    pub bootloader_size: u32,
    pub primary_slot_start: u32,
    pub primary_slot_size: u32,
    pub secondary_slot_start: u32,
    pub secondary_slot_size: u32,
    pub scratch_start: u32,
    pub scratch_size: u32,
    pub settings_start: u32,
    pub settings_size: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PartitionRegion {
    pub start: u32,
    pub size: u32,
}

impl PartitionRegion {
    pub const fn end(&self) -> u32 {
        self.start + self.size
    }
}

impl PartitionLayout {
    pub const FLASH_BASE: u32 = 0x0000_0000;
    pub const FLASH_SIZE: u32 = 1024 * 1024;
    pub const RAM_BASE: u32 = 0x2000_0000;
    pub const RAM_SIZE: u32 = 192 * 1024;

    pub const fn mcuboot_default() -> Self {
        Self {
            bootloader_start: Self::FLASH_BASE,
            bootloader_size: 64 * 1024,
            primary_slot_start: 0x0001_0000,
            primary_slot_size: 384 * 1024,
            secondary_slot_start: 0x0007_0000,
            secondary_slot_size: 384 * 1024,
            scratch_start: 0x000D_0000,
            scratch_size: 64 * 1024,
            settings_start: 0x000E_0000,
            settings_size: 128 * 1024,
        }
    }

    pub const fn bootloader_region(&self) -> PartitionRegion {
        PartitionRegion {
            start: self.bootloader_start,
            size: self.bootloader_size,
        }
    }

    pub const fn primary_slot_region(&self) -> PartitionRegion {
        PartitionRegion {
            start: self.primary_slot_start,
            size: self.primary_slot_size,
        }
    }

    pub const fn secondary_slot_region(&self) -> PartitionRegion {
        PartitionRegion {
            start: self.secondary_slot_start,
            size: self.secondary_slot_size,
        }
    }

    pub const fn scratch_region(&self) -> PartitionRegion {
        PartitionRegion {
            start: self.scratch_start,
            size: self.scratch_size,
        }
    }

    pub const fn settings_region(&self) -> PartitionRegion {
        PartitionRegion {
            start: self.settings_start,
            size: self.settings_size,
        }
    }

    pub const fn app_link_origin(&self) -> u32 {
        self.primary_slot_start
    }

    pub const fn app_link_length(&self) -> u32 {
        self.primary_slot_size
    }

    pub const fn bootloader_end(&self) -> u32 {
        self.bootloader_start + self.bootloader_size
    }

    pub const fn primary_slot_end(&self) -> u32 {
        self.primary_slot_start + self.primary_slot_size
    }

    pub const fn secondary_slot_end(&self) -> u32 {
        self.secondary_slot_start + self.secondary_slot_size
    }

    pub const fn scratch_end(&self) -> u32 {
        self.scratch_start + self.scratch_size
    }

    pub const fn settings_end(&self) -> u32 {
        self.settings_start + self.settings_size
    }

    pub fn validate(&self) -> Result<(), LayoutError> {
        if self.primary_slot_size != self.secondary_slot_size {
            return Err(LayoutError::SlotSizeMismatch);
        }

        if self.scratch_size == 0 {
            return Err(LayoutError::ScratchMissing);
        }

        if self.bootloader_start != Self::FLASH_BASE {
            return Err(LayoutError::BootloaderOffsetInvalid);
        }

        if self.bootloader_end() > self.primary_slot_start
            || self.primary_slot_end() > self.secondary_slot_start
            || self.secondary_slot_end() > self.scratch_start
            || self.scratch_end() > self.settings_start
        {
            return Err(LayoutError::RegionOverlap);
        }

        if self.settings_end() > Self::FLASH_BASE + Self::FLASH_SIZE {
            return Err(LayoutError::FlashOverflow);
        }

        Ok(())
    }
}

pub const MCU_BOOT_LAYOUT: PartitionLayout = PartitionLayout::mcuboot_default();
pub const BOOTLOADER_REGION: PartitionRegion = MCU_BOOT_LAYOUT.bootloader_region();
pub const PRIMARY_SLOT_REGION: PartitionRegion = MCU_BOOT_LAYOUT.primary_slot_region();
pub const SECONDARY_SLOT_REGION: PartitionRegion = MCU_BOOT_LAYOUT.secondary_slot_region();
pub const SCRATCH_REGION: PartitionRegion = MCU_BOOT_LAYOUT.scratch_region();
pub const SETTINGS_REGION: PartitionRegion = MCU_BOOT_LAYOUT.settings_region();

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LayoutError {
    BootloaderOffsetInvalid,
    FlashOverflow,
    RegionOverlap,
    ScratchMissing,
    SlotSizeMismatch,
}
