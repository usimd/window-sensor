use embassy_nrf::Peri;
use embassy_nrf::gpio::{Flex, OutputDrive, Pin, Pull};
use embassy_nrf::twim::{Error, Twim};
use embassy_time::{Duration, Instant, block_for};
use embedded_hal::i2c::{ErrorType, I2c, Operation};

pub const TRANSACTION_TIMEOUT: Duration = Duration::from_millis(50);

const HALF_PERIOD: Duration = Duration::from_micros(5);
const STRETCH_LIMIT: Duration = Duration::from_millis(2);

/// Blocking TWIM whose transactions fail with `Error::Timeout` instead of hanging on a stuck bus.
pub struct TimeoutTwim<'d> {
    twim: Twim<'d>,
    timeout: Duration,
}

impl<'d> TimeoutTwim<'d> {
    pub fn new(twim: Twim<'d>, timeout: Duration) -> Self {
        Self { twim, timeout }
    }
}

impl ErrorType for TimeoutTwim<'_> {
    type Error = Error;
}

impl I2c for TimeoutTwim<'_> {
    fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        self.twim
            .blocking_transaction_timeout(address, operations, self.timeout)
    }
}

/// Bus state seen before and after [`recover_bus`].
#[derive(Clone, Copy, defmt::Format)]
pub struct RecoveryReport {
    pub scl_before: bool,
    pub sda_before: bool,
    pub clocks: u8,
    pub scl_after: bool,
    pub sda_after: bool,
}

impl RecoveryReport {
    pub fn idle(&self) -> bool {
        self.scl_after && self.sda_after
    }
}

/// Clocks out a target stuck mid-transfer (up to 9 SCL pulses) and issues a STOP.
/// Must run before the pins are handed to TWIM.
pub fn recover_bus(scl: Peri<'_, impl Pin>, sda: Peri<'_, impl Pin>, pull: Pull) -> RecoveryReport {
    let mut scl = Flex::new(scl);
    let mut sda = Flex::new(sda);
    scl.set_high();
    sda.set_high();
    scl.set_as_input_output(pull, OutputDrive::Standard0Disconnect1);
    sda.set_as_input_output(pull, OutputDrive::Standard0Disconnect1);
    block_for(HALF_PERIOD);

    let scl_before = scl.is_high();
    let sda_before = sda.is_high();

    let mut clocks = 0;
    while sda.is_low() && clocks < 9 {
        scl.set_low();
        block_for(HALF_PERIOD);
        release_scl(&mut scl);
        clocks += 1;
    }

    // STOP: SDA rises while SCL is high. Pull SDA low only while SCL is low to avoid a START.
    scl.set_low();
    block_for(HALF_PERIOD);
    sda.set_low();
    block_for(HALF_PERIOD);
    release_scl(&mut scl);
    sda.set_high();
    block_for(HALF_PERIOD);

    let report = RecoveryReport {
        scl_before,
        sda_before,
        clocks,
        scl_after: scl.is_high(),
        sda_after: sda.is_high(),
    };
    scl.set_as_disconnected();
    sda.set_as_disconnected();
    report
}

fn release_scl(scl: &mut Flex<'_>) {
    scl.set_high();
    let deadline = Instant::now() + STRETCH_LIMIT;
    while scl.is_low() && Instant::now() < deadline {}
    block_for(HALF_PERIOD);
}
