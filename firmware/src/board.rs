//! Pin assignments for NORA-B216 module (nRF54L10)
//!
//! Derived from PCB netlist (window-sensor.kicad_pcb) — this is the source of truth.
//!
//! NORA-B216 module pin → nRF54L10 GPIO mapping (from UBX-24071142 product spec):
//!   A3 (I2C_SCL)  → P1.02
//!   B4 (I2C_SDA)  → P1.03
//!   A5 (GPIO_1)   → P1.04  → /HALL (TMAG5273 INT)
//!   A6 (GPIO_2)   → P1.05  → /MEMS_INT1 (LIS2DTW12 INT1)
//!   C8 (GPIO_3)   → P1.06  → /MEMS_INT2 (LIS2DTW12 INT2)
//!   H9 (GPIO_7)   → P1.10  → LED D2 (Red) anode
//!   J8 (GPIO_8)   → P1.08  → LED D1 (Blue) anode
//!   D2 (UART_TX)  → P1.01  → UART TX (debug telemetry)
//!
//! LEDs are active-high: GPIO drives anode, cathode through resistor to GND.

use embassy_nrf::peripherals;

// I2C bus (shared: TMAG5273 @ 0x22, LIS2DTW12 @ 0x19, SHT4x @ 0x44)
pub type I2cScl = peripherals::P1_02;
pub type I2cSda = peripherals::P1_03;

// Hall sensor interrupt (active-low, wake-from-sleep trigger)
pub type HallInt = peripherals::P1_04;

// MEMS accelerometer interrupts
pub type MemsInt1 = peripherals::P1_05;
pub type MemsInt2 = peripherals::P1_06;

// LEDs (active-high drive)
pub type LedBlue = peripherals::P1_08;
pub type LedRed = peripherals::P1_10;

// Debug UART TX
pub type UartTx = peripherals::P1_01;
