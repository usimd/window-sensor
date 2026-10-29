//! Sensor, pull-up and interrupt-line checks for the bring-up/factory test.
//! Every result is emitted as a `MEAS key=value` line for scripts/factory_test.py.

use defmt::info;
use embassy_nrf::Peri;
use embassy_nrf::gpio::{Flex, Input, OutputDrive, Pin, Pull};
use embassy_time::{Duration, Instant, Timer, block_for};
use embedded_hal::i2c::I2c;

use crate::i2c::TimeoutTwim;

type Bus<'a, 'd> = &'a mut TimeoutTwim<'d>;

fn write_reg(bus: Bus<'_, '_>, addr: u8, reg: u8, val: u8) -> Result<(), ()> {
    bus.write(addr, &[reg, val]).map_err(|_| ())
}

fn read_regs(bus: Bus<'_, '_>, addr: u8, reg: u8, buf: &mut [u8]) -> Result<(), ()> {
    bus.write_read(addr, &[reg], buf).map_err(|_| ())
}

async fn wait_level(pin: &Input<'_>, high: bool, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if pin.is_high() == high {
            return true;
        }
        Timer::after_micros(200).await;
    }
    pin.is_high() == high
}

fn magnitude(v: [i32; 3]) -> i32 {
    let sq: f32 = v.iter().map(|c| (*c as f32) * (*c as f32)).sum();
    libm::sqrtf(sq) as i32
}

/// Drives the line low, releases it without internal pull and checks it rises: only an external
/// pull-up (R1/R2, 16.9 kΩ) charges the bus capacitance within microseconds.
pub fn external_pullup(pin: Peri<'_, impl Pin>) -> bool {
    let mut line = Flex::new(pin);
    line.set_low();
    line.set_as_output(OutputDrive::Standard);
    block_for(Duration::from_micros(20));
    line.set_as_input(Pull::None);
    block_for(Duration::from_micros(10));
    let high = line.is_high();
    line.set_as_disconnected();
    high
}

/// Puts every sensor into its lowest-power state; errors are ignored (sensor may be absent).
pub fn all_to_sleep(bus: Bus<'_, '_>) {
    let _ = write_reg(
        bus,
        tmag5273::ADDR,
        tmag5273::DEVICE_CONFIG_2,
        tmag5273::MODE_SLEEP,
    );
    let _ = write_reg(bus, lis2dtw12::ADDR, lis2dtw12::CTRL1, 0x00);
}

pub mod lis2dtw12 {
    use super::*;

    pub const ADDR: u8 = 0x19;
    const WHO_AM_I: u8 = 0x0F;
    pub const CTRL1: u8 = 0x20;
    const CTRL2: u8 = 0x21;
    const CTRL3: u8 = 0x22;
    const CTRL4_INT1: u8 = 0x23;
    const CTRL5_INT2: u8 = 0x24;
    const CTRL6: u8 = 0x25;
    const STATUS: u8 = 0x27;
    const OUT_X_L: u8 = 0x28;
    const CTRL7: u8 = 0x3F;
    const EXPECTED_ID: u8 = 0x44;
    // 14-bit high-performance sensitivity in µg/LSB.
    const SENS_2G_UG: i32 = 244;
    const SENS_4G_UG: i32 = 488;

    async fn sample(bus: Bus<'_, '_>) -> Result<[i32; 3], ()> {
        let mut status = [0u8];
        for _ in 0..50 {
            read_regs(bus, ADDR, STATUS, &mut status)?;
            if status[0] & 0x01 != 0 {
                break;
            }
            Timer::after_millis(2).await;
        }
        let mut b = [0u8; 6];
        read_regs(bus, ADDR, OUT_X_L, &mut b)?;
        Ok([0, 2, 4].map(|i| i32::from(i16::from_le_bytes([b[i], b[i + 1]]) >> 2)))
    }

    async fn average(bus: Bus<'_, '_>, n: i32) -> Result<[i32; 3], ()> {
        let mut acc = [0i32; 3];
        for _ in 0..n {
            let s = sample(bus).await?;
            for (a, v) in acc.iter_mut().zip(s) {
                *a += v;
            }
        }
        Ok(acc.map(|a| a / n))
    }

    /// Latched data-ready must raise the pin with the next sample and drop once the data is read.
    async fn drdy_on_pin(bus: Bus<'_, '_>, pin: &Input<'_>) -> Result<bool, ()> {
        let mut out = [0u8; 6];
        read_regs(bus, ADDR, OUT_X_L, &mut out)?;
        let rose = wait_level(pin, true, Duration::from_millis(100)).await;
        read_regs(bus, ADDR, OUT_X_L, &mut out)?;
        let fell = wait_level(pin, false, Duration::from_millis(5)).await;
        Ok(rose && fell)
    }

    async fn run(bus: Bus<'_, '_>, int1: &Input<'_>, int2: &Input<'_>) -> Result<bool, ()> {
        write_reg(bus, ADDR, CTRL2, 0x40)?; // soft reset
        Timer::after_millis(5).await;
        write_reg(bus, ADDR, CTRL2, 0x0C)?; // BDU, register auto-increment
        write_reg(bus, ADDR, CTRL6, 0x04)?; // ±2 g, low noise
        write_reg(bus, ADDR, CTRL1, 0x44)?; // 50 Hz, high-performance
        Timer::after_millis(100).await;
        sample(bus).await?;
        let g_mg = magnitude(average(bus, 8).await?) * SENS_2G_UG / 1000;
        info!("MEAS lis_g_mg={}", g_mg);

        write_reg(bus, ADDR, CTRL7, 0x20)?; // interrupts enable
        write_reg(bus, ADDR, CTRL4_INT1, 0x01)?;
        let int1_ok = drdy_on_pin(bus, int1).await?;
        write_reg(bus, ADDR, CTRL4_INT1, 0x00)?;
        write_reg(bus, ADDR, CTRL5_INT2, 0x01)?;
        let int2_ok = drdy_on_pin(bus, int2).await?;
        write_reg(bus, ADDR, CTRL5_INT2, 0x00)?;
        info!("MEAS lis_int1={}", u8::from(int1_ok));
        info!("MEAS lis_int2={}", u8::from(int2_ok));

        // Self-test per ST AN5038: ±4 g, compare averages with and without positive ST.
        write_reg(bus, ADDR, CTRL6, 0x10)?;
        Timer::after_millis(100).await;
        sample(bus).await?;
        let nost = average(bus, 5).await?;
        write_reg(bus, ADDR, CTRL3, 0x40)?;
        Timer::after_millis(100).await;
        sample(bus).await?;
        let st = average(bus, 5).await?;
        write_reg(bus, ADDR, CTRL3, 0x00)?;
        let delta = [0, 1, 2].map(|i| (st[i] - nost[i]).abs() * SENS_4G_UG / 1000);
        info!("MEAS lis_st_x_mg={}", delta[0]);
        info!("MEAS lis_st_y_mg={}", delta[1]);
        info!("MEAS lis_st_z_mg={}", delta[2]);

        Ok(int1_ok && int2_ok)
    }

    pub async fn test(bus: Bus<'_, '_>, int1: &Input<'_>, int2: &Input<'_>) -> bool {
        let mut id = [0u8];
        if read_regs(bus, ADDR, WHO_AM_I, &mut id).is_err() {
            info!("MEAS lis_whoami=none");
            return false;
        }
        info!("MEAS lis_whoami={=u8:#04x}", id[0]);
        let ok = run(bus, int1, int2).await;
        let _ = write_reg(bus, ADDR, CTRL4_INT1, 0x00);
        let _ = write_reg(bus, ADDR, CTRL5_INT2, 0x00);
        let _ = write_reg(bus, ADDR, CTRL7, 0x00);
        let _ = write_reg(bus, ADDR, CTRL1, 0x00);
        id[0] == EXPECTED_ID && ok == Ok(true)
    }
}

pub mod tmag5273 {
    use super::*;

    pub const ADDR: u8 = 0x22;
    pub const DEVICE_CONFIG_2: u8 = 0x01;
    const SENSOR_CONFIG_1: u8 = 0x02;
    const INT_CONFIG_1: u8 = 0x08;
    const DEVICE_ID: u8 = 0x0D;
    const X_MSB_RESULT: u8 = 0x12;
    pub const MODE_SLEEP: u8 = 0x01;
    const MODE_CONTINUOUS: u8 = 0x02;
    const EXPECTED_MANUFACTURER: u16 = 0x5449;
    const MAGNET_WINDOW: Duration = Duration::from_secs(15);
    // Stop the magnet window early once the field moved this far (0.1 mT units).
    const MAGNET_DONE_DMT: i32 = 50;

    /// Field magnitude in 0.1 mT.
    fn field_dmt(bus: Bus<'_, '_>, range_mt: i32) -> Result<i32, ()> {
        let mut b = [0u8; 6];
        read_regs(bus, ADDR, X_MSB_RESULT, &mut b)?;
        let axes = [0, 2, 4]
            .map(|i| i32::from(i16::from_be_bytes([b[i], b[i + 1]])) * range_mt * 10 / 32768);
        Ok(magnitude(axes))
    }

    async fn run(bus: Bus<'_, '_>, hall: &Input<'_>, range_mt: i32) -> Result<bool, ()> {
        write_reg(bus, ADDR, SENSOR_CONFIG_1, 0x70)?; // X, Y, Z
        write_reg(bus, ADDR, INT_CONFIG_1, 0x84)?; // result interrupt, latched, INT pin
        write_reg(bus, ADDR, DEVICE_CONFIG_2, MODE_CONTINUOUS)?;
        let int_ok = wait_level(hall, false, Duration::from_millis(50)).await;
        info!("MEAS tmag_int={}", u8::from(int_ok));

        let mut baseline = 0;
        for _ in 0..10 {
            baseline += field_dmt(bus, range_mt)?;
            Timer::after_millis(20).await;
        }
        baseline /= 10;
        info!("MEAS tmag_baseline_dmt={}", baseline);

        info!("PROMPT magnet");
        let deadline = Instant::now() + MAGNET_WINDOW;
        let mut max_delta = 0;
        while Instant::now() < deadline && max_delta < MAGNET_DONE_DMT {
            max_delta = max_delta.max((field_dmt(bus, range_mt)? - baseline).abs());
            Timer::after_millis(20).await;
        }
        info!("MEAS tmag_magnet_delta_dmt={}", max_delta);
        Ok(int_ok)
    }

    pub async fn test(bus: Bus<'_, '_>, hall: &Input<'_>) -> bool {
        let mut id = [0u8; 3];
        if read_regs(bus, ADDR, DEVICE_ID, &mut id).is_err() {
            info!("MEAS tmag_manufacturer=none");
            return false;
        }
        let manufacturer = u16::from_le_bytes([id[1], id[2]]);
        info!("MEAS tmag_manufacturer={=u16:#06x}", manufacturer);
        info!("MEAS tmag_device_id={=u8:#04x}", id[0]);
        // DEVICE_ID.VER: 1 = x1 (±40 mT default range), 2 = x2 (±133 mT).
        let range_mt = if id[0] & 0x03 == 2 { 133 } else { 40 };
        let ok = run(bus, hall, range_mt).await;
        let _ = write_reg(bus, ADDR, INT_CONFIG_1, 0x01);
        let _ = write_reg(bus, ADDR, DEVICE_CONFIG_2, MODE_SLEEP);
        manufacturer == EXPECTED_MANUFACTURER && ok == Ok(true)
    }
}

pub mod sht4x {
    use super::*;

    pub const ADDR: u8 = 0x44;
    const SOFT_RESET: u8 = 0x94;
    const READ_SERIAL: u8 = 0x89;
    const MEASURE_HIGH_PRECISION: u8 = 0xFD;

    fn crc8(data: &[u8]) -> u8 {
        let mut crc = 0xFFu8;
        for byte in data {
            crc ^= byte;
            for _ in 0..8 {
                crc = if crc & 0x80 != 0 {
                    (crc << 1) ^ 0x31
                } else {
                    crc << 1
                };
            }
        }
        crc
    }

    /// Two CRC-protected 16-bit words.
    async fn command(bus: Bus<'_, '_>, cmd: u8, wait_ms: u64) -> Result<([u16; 2], bool), ()> {
        bus.write(ADDR, &[cmd]).map_err(|_| ())?;
        Timer::after_millis(wait_ms).await;
        let mut b = [0u8; 6];
        bus.read(ADDR, &mut b).map_err(|_| ())?;
        let crc_ok = crc8(&b[0..2]) == b[2] && crc8(&b[3..5]) == b[5];
        Ok((
            [
                u16::from_be_bytes([b[0], b[1]]),
                u16::from_be_bytes([b[3], b[4]]),
            ],
            crc_ok,
        ))
    }

    async fn run(bus: Bus<'_, '_>) -> Result<bool, ()> {
        bus.write(ADDR, &[SOFT_RESET]).map_err(|_| ())?;
        Timer::after_millis(2).await;
        let (serial, serial_crc) = command(bus, READ_SERIAL, 2).await?;
        let (raw, meas_crc) = command(bus, MEASURE_HIGH_PRECISION, 10).await?;
        let t_cdeg = -4500 + 17500 * i32::from(raw[0]) / 65535;
        let rh_cpct = (-600 + 12500 * i32::from(raw[1]) / 65535).clamp(0, 10000);
        let crc_ok = serial_crc && meas_crc;
        info!(
            "MEAS sht_serial={=u32:#010x}",
            (u32::from(serial[0]) << 16) | u32::from(serial[1])
        );
        info!("MEAS sht_crc_ok={}", u8::from(crc_ok));
        info!("MEAS sht_t_cdeg={}", t_cdeg);
        info!("MEAS sht_rh_cpct={}", rh_cpct);
        Ok(crc_ok)
    }

    pub async fn test(bus: Bus<'_, '_>) -> bool {
        match run(bus).await {
            Ok(ok) => ok,
            Err(()) => {
                info!("MEAS sht_serial=none");
                false
            }
        }
    }
}
