#![no_std]
#![no_main]
#![allow(dead_code)]

//! Hardware bring-up test for the window-sensor prototype.
//!
//! Run with: `just hwtest`  (flashes and streams RTT via probe-rs)
//!
//! Test sequence:
//!   ① SoC alive    – logs version + reset reason over RTT
//!   ② LED blink    – blue×3 then red×3 to verify GPIO drive
//!   ③ I2C probe    – WHO_AM_I / serial reads from TMAG5273, LIS2DTW12, SHT4x
//!                    blink blue = OK, red = FAIL per device
//!   ④ BTHome adv   – 10-burst advertisement (visible in any BTHome scanner app)
//!   ⑤ Summary      – long blue blink = all OK, long red blink = failures found

use bt_hci::cmd::SyncCmd;
use bt_hci::cmd::le::{LeSetAdvData, LeSetAdvEnable, LeSetAdvParams};
use bt_hci::param::{AddrKind, AdvChannelMap, AdvFilterPolicy, AdvKind, BdAddr, Duration};
use defmt::*;
use embassy_executor::Spawner;
use embassy_nrf::cracen;
use embassy_nrf::gpio::{Flex, Level, Output, OutputDrive, Pull};
use embassy_nrf::pac;
use embassy_nrf::twim;
use embassy_nrf::{bind_interrupts, peripherals};
use embassy_time::Timer;
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::vendor::ZephyrWriteBdAddr;
use nrf_sdc::{self as sdc, mpsl};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_reset as _}; // panic_reset: resets instead of halting (works without debugger)

mod board;
mod drivers;

use drivers::led::Led;

// I2C addresses from board.rs comments
const TMAG5273_ADDR: u8 = 0x22;
const LIS2DTW12_ADDR: u8 = 0x19;
const SHT4X_ADDR: u8 = 0x44;

const ADV_INTERVAL_MS: u32 = 20;
const ADV_BURST_COUNT: u8 = 10;
const ADV_BURST_WINDOW_MS: u64 = 100;

bind_interrupts!(struct Irqs {
    SERIAL20 => twim::InterruptHandler<peripherals::SERIAL20>;
    SERIAL30 => twim::InterruptHandler<peripherals::SERIAL30>;
    SWI00 => mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => mpsl::ClockInterruptHandler;
    RADIO_0 => mpsl::HighPrioInterruptHandler;
    TIMER10 => mpsl::HighPrioInterruptHandler;
    GRTC_3 => mpsl::HighPrioInterruptHandler;
});

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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // ── PERIPHERAL RESET GUARD (errata [105] workaround) ─────────────────────
    // pyocd flash issues a DAP (debug) reset which resets the CPU but NOT
    // peripheral registers. The TWIM FSM survives and stays stuck if errata
    // [105] was previously triggered. SYSRESETREQ (soft reset) resets all
    // peripherals. We do it once per cold-boot sequence, gated on RESETREAS:
    //   - sreq=0 → first run after DAP/POR reset → do soft reset to clear peripherals
    //   - sreq=1 → we just did the soft reset → proceed normally
    {
        let resetreas = pac::RESET.resetreas().read();
        // Clear the register so the next reset shows clean state
        pac::RESET.resetreas().write(|w| *w = resetreas);
        if !resetreas.sreq() {
            // Not a soft reset: trigger SYSRESETREQ to reset all peripheral FSMs
            cortex_m::peripheral::SCB::sys_reset(); // diverges, resets the SoC
        }
        // If we get here: resetreas.sreq() == true → peripherals are clean
    }

    let p = embassy_nrf::init(Default::default());

    // --- CLOCK DIAGNOSTIC (Opus RCA: verify PLL/HSFLL actually reaches peripherals) ---
    // OSCILLATORS_S base: 0x5012_0000
    // pll().freq() at +0x800, pll().currentfreq() at +0x804
    let osc_freq    = unsafe { (0x5012_0800usize as *const u32).read_volatile() };
    let osc_curfreq = unsafe { (0x5012_0804usize as *const u32).read_volatile() };
    // CLOCK_S base: 0x5010_e000
    // events_xostarted: +0x100 = 0x5010_e100
    // events_pllstarted: +0x104 = 0x5010_e104
    let clk_events_xo  = unsafe { (0x5010_e100usize as *const u32).read_volatile() };
    let clk_events_pll = unsafe { (0x5010_e104usize as *const u32).read_volatile() };
    info!("BEFORE xo/pll: OSC.pll.freq=0x{:08x} currentfreq=0x{:08x}", osc_freq, osc_curfreq);
    info!("BEFORE xo/pll: CLOCK.events_xostarted=0x{:08x} events_pllstarted=0x{:08x}", clk_events_xo, clk_events_pll);

    // Errata [39]: PLL before XO
    pac::CLOCK.events_pllstarted().write_value(0);
    pac::CLOCK.tasks_pllstart().write_value(1);
    pac::CLOCK.events_xostarted().write_value(0);
    pac::CLOCK.tasks_xostart().write_value(1);
    let mut timeout = 100_000u32;
    while pac::CLOCK.events_xostarted().read() == 0 && timeout > 0 { timeout -= 1; }
    let mut timeout = 100_000u32;
    while pac::CLOCK.events_pllstarted().read() == 0 && timeout > 0 { timeout -= 1; }

    let osc_freq2    = unsafe { (0x5012_0800usize as *const u32).read_volatile() };
    let osc_curfreq2 = unsafe { (0x5012_0804usize as *const u32).read_volatile() };
    let clk_events_xo2  = unsafe { (0x5010_e100usize as *const u32).read_volatile() };
    let clk_events_pll2 = unsafe { (0x5010_e104usize as *const u32).read_volatile() };
    info!("AFTER  xo/pll: OSC.pll.freq=0x{:08x} currentfreq=0x{:08x}", osc_freq2, osc_curfreq2);
    info!("AFTER  xo/pll: CLOCK.events_xostarted=0x{:08x} events_pllstarted=0x{:08x}", clk_events_xo2, clk_events_pll2);
    // currentfreq: 0x01=CK128M running, 0x03=CK64M running, 0x00=HSFLL not running!

    // ── PRE-TWIM LINE STATE CHECK ─────────────────────────────────────────────
    // Configure P1.02 (SCL) and P1.03 (SDA) as push-pull OUTPUT HIGH BEFORE
    // Twim::new() takes ownership. If IN reads 0 here → slave is pulling the line
    // LOW externally (TWIM is not involved yet).
    // PIN_CNF: bit0=dir(1=output), bits[11:8]=drive(0b0000=S0S1 push-pull)
    {
        const P1: usize = 0x500d_8200;
        unsafe {
            ((P1 + 0x04) as *mut u32).write_volatile((1 << 2) | (1 << 3)); // OUTSET: latch HIGH
            ((P1 + 0x80 + 2*4) as *mut u32).write_volatile(0x0000_0001);   // P1.02: out, S0S1 push-pull
            ((P1 + 0x80 + 3*4) as *mut u32).write_volatile(0x0000_0001);   // P1.03: out, S0S1 push-pull
            cortex_m::asm::dsb();
            for _ in 0..5_000u32 { cortex_m::asm::nop(); }
            let p1_out = ((P1 + 0x00) as *const u32).read_volatile();
            let p1_in  = ((P1 + 0x0c) as *const u32).read_volatile();
            info!("PRE-TWIM push-pull-HIGH: OUT=0x{:08x} IN=0x{:08x} SCL_in={} SDA_in={}  (0=slave pulling low)",
                p1_out, p1_in, (p1_in >> 2) & 1, (p1_in >> 3) & 1);
        }
    }
    // embassy Twim::new() will reconfigure P1.02 and P1.03 as open-drain (correct).

    static TWIM_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let mut i2c = embassy_nrf::twim::Twim::new(
        p.SERIAL20, Irqs, p.P1_03, p.P1_02,
        twim::Config::default(),
        TWIM_BUF.init([0u8; 16]),
    );
    let p1_in = unsafe { (0x500d_820cusize as *const u32).read_volatile() }; // P1.IN (+0x0c), not DIR (+0x10)
    info!("POST-TWIM open-drain: IN=0x{:08x} SCL_in={} SDA_in={}",
        p1_in, (p1_in >> 2) & 1, (p1_in >> 3) & 1);

    // ── SPU20 PERIPH diagnostic ───────────────────────────────────────────────
    // SPU20_S base: 0x500c_0000; PERIPH[n] at +0x500 + n*4
    // Check which PERIPH slots are present (bit31) and their secattr (bit4)
    {
        let spu20: *const u32 = 0x500c_0500 as *const u32;
        info!("SPU20 PERIPH[0..7] (present=bit31 secattr=bit4 dmasec=bit5):");
        for i in 0..8usize {
            let val = unsafe { spu20.add(i).read_volatile() };
            let present = (val >> 31) & 1;
            let secattr = (val >> 4) & 1;
            let dmasec  = (val >> 5) & 1;
            let secmap  = val & 3;
            info!("  PERIPH[{}] = 0x{:08x}  present={} secattr={} dmasec={} secmap={}", 
                i, val, present, secattr, dmasec, secmap);
        }
    }

    // ── BARE-METAL TWIM: comprehensive event + DMA monitoring ─────────────────
    // Wait 100 ms to let all I2C slaves complete their power-on / measurement
    // initialization before we attempt a transaction (TMAG5273 conversion ~5 ms).
    Timer::after_millis(100).await;
    // Now also polling events_dma.tx.ready (TXSTARTED = 0x168) and rx.ready (0x14c)
    {
        const TWIM: usize = 0x500c_6000;
        unsafe fn rd(base: usize, off: usize) -> u32 { ((base + off) as *const u32).read_volatile() }
        unsafe fn wr(base: usize, off: usize, v: u32) { ((base + off) as *mut u32).write_volatile(v); cortex_m::asm::dsb(); }

        // Use static mut buffers to guarantee placement in RAM (.data section)
        static mut TX_BUF: [u8; 1] = [0x0Du8];
        static mut RX_BUF: [u8; 1] = [0u8];

        unsafe {
            // P1_S base = 0x500d_8200 (NOT 0x500d_8000!)
            // pin_cnf(n) = base + 0x80 + n*4
            // in_()      = base + 0x0c
            // out()      = base + 0x0  (same as base!)
            const P1: usize = 0x500d_8200;
            let pincnf2 = rd(P1, 0x80 + 2*4); // P1.02 = SCL  → 0x500d_8288
            let pincnf3 = rd(P1, 0x80 + 3*4); // P1.03 = SDA  → 0x500d_828c
            // Expected after embassy Twim::new(): dir=OUTPUT(1)=bit0, drive1=D(2)=bits10-11 → 0x0000_0801
            info!("GPIO P1.02(SCL) PIN_CNF=0x{:08x}  P1.03(SDA) PIN_CNF=0x{:08x}  (expect 0x00000801)",
                pincnf2, pincnf3);
            let p1_out = rd(P1, 0x00); // OUT register (latch)
            let p1_in  = rd(P1, 0x0c); // IN register  (actual pin state)
            info!("P1 OUT=0x{:08x}  IN=0x{:08x}  SCL_out={} SDA_out={}  SCL_in={} SDA_in={}",
                p1_out, p1_in,
                (p1_out >> 2) & 1, (p1_out >> 3) & 1,
                (p1_in  >> 2) & 1, (p1_in  >> 3) & 1);

            // Read PSEL BEFORE disable (from embassy init)
            let psel_scl_before = rd(TWIM, 0x600);
            let psel_sda_before = rd(TWIM, 0x604);
            info!("PSEL before DISABLE: SCL=0x{:08x} SDA=0x{:08x}", psel_scl_before, psel_sda_before);

            // DISABLE → set PSEL (correct order) → ENABLE
            wr(TWIM, 0x500, 0); // DISABLE
            cortex_m::asm::dsb();
            wr(TWIM, 0x600, 0x0000_0022); // PSEL.SCL = P1.02 (while disabled)
            wr(TWIM, 0x604, 0x0000_0023); // PSEL.SDA = P1.03 (while disabled)
            wr(TWIM, 0x524, 0x0198_0000); // FREQUENCY = 100kHz
            wr(TWIM, 0x588, 0x0000_0022); // ADDRESS = 0x22
            cortex_m::asm::dsb();
            wr(TWIM, 0x500, 6); // ENABLE

            // KEY TEST: fire tasks_resume immediately after ENABLE
            // TWIM holds SCL+SDA LOW in its initial state — resume releases them to IDLE
            wr(TWIM, 0x010, 1); // tasks_resume
            // Small delay for lines to settle
            for _ in 0..100u32 { cortex_m::asm::nop(); }

            // Read PSEL AFTER enable to verify it stuck
            let psel_scl_after = rd(TWIM, 0x600);
            let psel_sda_after = rd(TWIM, 0x604);
            let en = rd(TWIM, 0x500);
            info!("PSEL after ENABLE+resume: SCL=0x{:08x} SDA=0x{:08x} EN={}", psel_scl_after, psel_sda_after, en);

            // Check if lines are now HIGH after resume
            let p1_in2 = rd(P1, 0x0c);
            info!("P1 IN after resume=0x{:08x}  SCL(bit2)={} SDA(bit3)={}",
                p1_in2, (p1_in2 >> 2) & 1, (p1_in2 >> 3) & 1);

            // DISABLE TWIM and check lines — if they go HIGH, TWIM was holding them LOW
            wr(TWIM, 0x500, 0); // DISABLE
            // Also clear PSEL to fully disconnect peripheral from pins
            wr(TWIM, 0x600, 0x8000_0000); // PSEL.SCL = DISCONNECTED
            wr(TWIM, 0x604, 0x8000_0000); // PSEL.SDA = DISCONNECTED
            // Force GPIO output HIGH via OUTSET (pins should already be 1 per OUT latch)
            wr(P1, 0x04, (1 << 2) | (1 << 3)); // OUTSET bits 2&3
            for _ in 0..500u32 { cortex_m::asm::nop(); }
            let p1_out3 = rd(P1, 0x00);
            let p1_in3  = rd(P1, 0x0c);
            info!("After DISABLE+PSEL_clr+OUTSET: OUT=0x{:08x} IN=0x{:08x}  SCL_out={} SCL_in={} SDA_out={} SDA_in={}",
                p1_out3, p1_in3,
                (p1_out3 >> 2) & 1, (p1_in3 >> 2) & 1,
                (p1_out3 >> 3) & 1, (p1_in3 >> 3) & 1);
            // If lines are still LOW → I2C slave(s) stuck. Send 9 SCL pulses to recover bus.
            let scl_in = (p1_in3 >> 2) & 1;
            let sda_in = (p1_in3 >> 3) & 1;
            if scl_in == 0 || sda_in == 0 {
                info!("BUS STUCK (SCL={} SDA={}) — sending bus recovery sequence", scl_in, sda_in);
                // Configure SCL as GPIO output push-pull (overrides any slave stretch), SDA as output
                wr(P1, 0x80 + 2*4, 0x0000_0001); // P1.02 SCL: OUTPUT, drive0=S (push-pull, overrides stretch)
                wr(P1, 0x80 + 3*4, 0x0000_0000); // P1.03 SDA: INPUT
                // Drive SCL HIGH first (override any slave holding it low)
                wr(P1, 0x04, 1 << 2); // OUTSET SCL → HIGH (push-pull, releases slave stretch)
                for _ in 0..200u32 { cortex_m::asm::nop(); }
                // 9 clock pulses
                for _ in 0..9u32 {
                    wr(P1, 0x08, 1 << 2); // OUTCLR SCL → LOW
                    for _ in 0..100u32 { cortex_m::asm::nop(); }
                    wr(P1, 0x04, 1 << 2); // OUTSET SCL → HIGH
                    for _ in 0..100u32 { cortex_m::asm::nop(); }
                    let sda_now = (rd(P1, 0x0c) >> 3) & 1;
                    if sda_now == 1 { info!("  SDA released after clocks"); }
                }
                // Generate STOP: SDA LOW→HIGH while SCL HIGH
                wr(P1, 0x80 + 3*4, 0x0000_0001); // P1.03 SDA: OUTPUT push-pull
                wr(P1, 0x08, 1 << 3); // SDA LOW
                for _ in 0..100u32 { cortex_m::asm::nop(); }
                wr(P1, 0x04, 1 << 3); // SDA HIGH (STOP condition)
                for _ in 0..500u32 { cortex_m::asm::nop(); }
                let p1_in4 = rd(P1, 0x0c);
                info!("After bus recovery: SCL_in={} SDA_in={}", (p1_in4 >> 2) & 1, (p1_in4 >> 3) & 1);
            }
            // Restore PIN_CNF to open-drain OUTPUT (same as embassy init)
            wr(P1, 0x80 + 2*4, 0x0000_0801); // SCL P1.02: dir=OUTPUT, drive1=D
            wr(P1, 0x80 + 3*4, 0x0000_0801); // SDA P1.03: dir=OUTPUT, drive1=D

            // ── PULL-UP DIAGNOSTIC ───────────────────────────────────────────────────────
            // Distinguishes "missing/broken pull-up" from "slave actively driving SDA LOW".
            // 1) Push-pull HIGH both lines (charges capacitance regardless of pull-up).
            // 2) Switch to INPUT. If pull-up works → stays HIGH indefinitely.
            //    If pull-up missing → capacitance discharges slowly (no current source).
            //    If slave driving → immediate or fast low.
            // Read after 100µs, 1ms, 10ms. τ=16.9kΩ×100pF=1.69µs so 100µs >> 59×τ ← fully settled.
            // Drive HIGH push-pull (S0S1 standard) to charge both lines before test.
            wr(P1, 0x08, (1<<2)|(1<<3));     // OUTCLR (pre-clear latch)
            wr(P1, 0x80 + 2*4, 0x0000_0001); // SCL push-pull output
            wr(P1, 0x80 + 3*4, 0x0000_0001); // SDA push-pull output
            wr(P1, 0x04, (1<<2)|(1<<3));     // OUTSET both HIGH
            for _ in 0..640u32 { cortex_m::asm::nop(); } // 10µs settle
            let pp_hi = rd(P1, 0x0c);
            // Switch to INPUT (release both to pull-up / floating)
            wr(P1, 0x80 + 2*4, 0x0000_0000); // SCL INPUT
            wr(P1, 0x80 + 3*4, 0x0000_0000); // SDA INPUT
            // Read immediately, after 100µs, after 1ms, after 10ms
            let inp_0us  = rd(P1, 0x0c);
            for _ in 0..6400u32 { cortex_m::asm::nop(); }  // ~100µs at 64MHz
            let inp_100us = rd(P1, 0x0c);
            for _ in 0..57600u32 { cortex_m::asm::nop(); } // ~900µs more (total ~1ms)
            let inp_1ms  = rd(P1, 0x0c);
            for _ in 0..576000u32 { cortex_m::asm::nop(); } // ~9ms more (total ~10ms)
            let inp_10ms = rd(P1, 0x0c);
            info!("PULL-UP DIAG: push-pull-HIGH SCL={} SDA={} (expect 1 1)",
                (pp_hi >> 2) & 1, (pp_hi >> 3) & 1);
            info!("  INPUT 0µs:   SCL={} SDA={}  (cap still charged, expect 1 1)",
                (inp_0us >> 2) & 1, (inp_0us >> 3) & 1);
            info!("  INPUT 100µs: SCL={} SDA={}  (59×τ_RC; pull-up present → 1 1; missing → may drift)",
                (inp_100us >> 2) & 1, (inp_100us >> 3) & 1);
            info!("  INPUT 1ms:   SCL={} SDA={}  (590×τ_RC; pull-up present → 1 1; missing → likely 0)",
                (inp_1ms >> 2) & 1, (inp_1ms >> 3) & 1);
            info!("  INPUT 10ms:  SCL={} SDA={}  (5900×τ_RC; pull-up present → 1 1; missing → 0)",
                (inp_10ms >> 2) & 1, (inp_10ms >> 3) & 1);
            // Restore push-pull HIGH for the bit-bang section that follows
            wr(P1, 0x04, (1<<2)|(1<<3));
            wr(P1, 0x80 + 2*4, 0x0000_0801); // SCL open-drain output
            wr(P1, 0x80 + 3*4, 0x0000_0801); // SDA open-drain output

            // ── BIT-BANG I2C WITH ACTUAL REGISTER READS ─────────────────────────────────
            // SCL = P1.02 (bit 2), SDA = P1.03 (bit 3).
            // SCL: push-pull output (PIN_CNF 0x0000_0001 = dir=out, S0S1 standard drive).
            //   This avoids pull-up speed as a variable — MCU drives SCL both edges.
            //   Overrides slave clock-stretch electrically; tells us if stretch is involved.
            // SDA TX: push-pull output 0 (drive LOW) or INPUT (0x0000_0000, release to pull-up).
            //   Switching between OUTPUT and INPUT for SDA is standard bit-bang I2C.
            //   This eliminates ambiguity about the open-drain drive mode encoding.
            // Bit rate: half-bit ≈ 50µs @ 64MHz (= ~10kHz I2C).  Very conservative to rule
            //   out pull-up RC rise-time issues (at 10kHz, 16.9kΩ × any realistic PCB cap is fine).
            {
                macro_rules! half_bit { () => { for _ in 0..3200u32 { cortex_m::asm::nop(); } } }
                // SCL: push-pull output. No INPUT needed (MCU always drives SCL).
                macro_rules! scl_lo { () => {
                    wr(P1, 0x80+2*4, 0x0000_0001); // out push-pull
                    wr(P1, 0x08, 1<<2);             // OUTCLR → LOW
                    half_bit!();
                } }
                macro_rules! scl_hi { () => {
                    wr(P1, 0x80+2*4, 0x0000_0001); // out push-pull
                    wr(P1, 0x04, 1<<2);             // OUTSET → HIGH
                    half_bit!();
                } }
                // SDA: switch between push-pull-0 (drive LOW) and INPUT (release to pull-up).
                macro_rules! sda_lo { () => {
                    wr(P1, 0x08, 1<<3);             // OUTCLR → 0 (pre-set latch before switching mode)
                    wr(P1, 0x80+3*4, 0x0000_0001);  // dir=out push-pull → drives LOW
                } }
                macro_rules! sda_rel { () => {       // release SDA: switch to input
                    wr(P1, 0x80+3*4, 0x0000_0000);  // dir=input → high-Z, pull-up drives HIGH
                } }
                macro_rules! sda_read { () => { (rd(P1, 0x0c) >> 3) & 1 } }

                // Forced bus recovery:
                // OLD BUG: exit on sda_released=1 after just 1-2 clocks is a false positive.
                // TMAG5273 has SDA=HIGH for certain bit positions but is still mid-transaction.
                // The STOP then provides a final inadvertent SCL clock (from scl_hi!) which
                // advances TMAG to the *next* bit (=0), so it immediately drives SDA=0 after STOP.
                //
                // FIX: Generate ALL 256 clocks (no early exit). Count trailing HIGH SCL periods
                // to confirm the slave has truly released SDA. Then generate 3 STOPs in a row,
                // polling SDA for 2ms after each one to see if/when the slave re-drives.
                // Key: the slave sees SDA=INPUT (high-Z = NACK) on every ACK slot, which tells
                // it to stop sending data and release SDA.
                {
                    sda_rel!();   // SDA = INPUT (NACK on all ACK slots)
                    scl_lo!();
                    let mut trailing_high = 0u32;
                    let mut last_low_n = 0u32;
                    for n in 0..256u32 {
                        scl_hi!();
                        let s = sda_read!();
                        scl_lo!();
                        if s == 1 { trailing_high += 1; }
                        else       { trailing_high = 0; last_low_n = n; }
                    }
                    info!("recovery 256clks: trailing_high={} last_low_at={}",
                        trailing_high, last_low_n);

                    // 3 STOP conditions. After each one, poll SDA for ~2ms.
                    // first_low records the iteration (≈250ns each) at which SDA goes LOW.
                    // first_low=None → SDA stayed HIGH = bus free after this STOP.
                    for stop_n in 0u32..3 {
                        sda_lo!(); half_bit!(); // SDA LOW while SCL LOW (not START)
                        scl_hi!(); half_bit!(); // SCL HIGH
                        sda_rel!();             // SDA→INPUT: STOP condition
                        let mut first_low: Option<u32> = None;
                        for t in 0..8000u32 { // ~2ms at ~250ns/iter
                            cortex_m::asm::nop();
                            cortex_m::asm::nop();
                            cortex_m::asm::nop();
                            if sda_read!() == 0 {
                                if first_low.is_none() { first_low = Some(t); }
                                break;
                            }
                        }
                        half_bit!();
                        let sda_f = sda_read!();
                        let scl_f = (rd(P1, 0x0c) >> 2) & 1;
                        info!("  STOP#{}: SCL={} SDA={} first_low_at={:?}",
                            stop_n, scl_f, sda_f, first_low);
                    }
                    let pin_r = rd(P1, 0x0c);
                    info!("forced_recovery done: SCL={} SDA={}",
                        (pin_r>>2)&1, (pin_r>>3)&1);
                }

                // SCL HIGH, SDA INPUT = idle bus.
                //
                // PASSIVE wait for bus idle: hold SCL=HIGH push-pull, SDA=INPUT, and
                // ONLY poll SDA — do NOT clock SCL. Clocking SCL in a tight loop triggers
                // TMAG5273 (stuck in continuous-meas mode from previous session) to keep
                // outputting data, preventing SDA from going HIGH.
                //
                // TMAG5273 in continuous-meas mode completes a conversion every ~8.5s, briefly
                // releasing the bus. We must wait passively for that window.
                //
                // Each iter: ~5×nop + MMIO read ≈ ~150ns @ 64MHz.
                // 20s timeout = 20_000_000_000ns / 150ns ≈ 133_000_000 iters (too many).
                // Use 15s = 100_000_000 iters of 150ns each ≈ 15s.
                //
                // Record the FIRST iter where SDA=1 (bus free) and also sample the SDA bit
                // pattern during the initial 256 clocks to distinguish "slave transmitting"
                // from "slave stretching".
                sda_rel!();
                wr(P1, 0x80+2*4, 0x0000_0001); wr(P1, 0x04, 1<<2); // SCL HIGH push-pull, hold there
                let mut sda_first_high: Option<u32> = None;
                let mut sda_sample_0ms  = 0u32;
                let mut sda_sample_1s   = 0u32;
                let mut sda_sample_5s   = 0u32;
                let mut sda_sample_10s  = 0u32;
                // 100M iters × ~150ns ≈ 15s max wait
                for i in 0..100_000_000u32 {
                    cortex_m::asm::nop();
                    cortex_m::asm::nop();
                    cortex_m::asm::nop();
                    let sda = sda_read!();
                    if i == 0       { sda_sample_0ms  = sda; }
                    if i == 6_666_667 { sda_sample_1s = sda; }  // ~1s
                    if i == 33_333_333 { sda_sample_5s = sda; } // ~5s
                    if i == 66_666_667 { sda_sample_10s = sda; } // ~10s
                    if sda == 1 && sda_first_high.is_none() {
                        sda_first_high = Some(i);
                        break; // Bus went idle — proceed to transactions immediately
                    }
                }
                info!("passive_wait: SDA@0ms={} @1s={} @5s={} @10s={} first_high={:?}",
                    sda_sample_0ms, sda_sample_1s, sda_sample_5s, sda_sample_10s, sda_first_high);

                // ── LOGIC2 ARM WINDOW ─────────────────────────────────────────
                // Bus is idle. Hold here 10 s so Logic2 can be (re)armed with
                // a "SDA↓ while SCL=HIGH" trigger before I2C transactions start.
                info!("=== ARM LOGIC2 NOW (10s window) ===");
                Timer::after_secs(10).await;
                info!("=== logic2 window expired, starting I2C ===");

                // Check bus idle before transactions
                sda_rel!();
                wr(P1, 0x04, 1<<2); wr(P1, 0x80+2*4, 0x0000_0001); // SCL HIGH
                half_bit!(); half_bit!();
                let idle = rd(P1, 0x0c);
                info!("bb pre-tx idle: SCL={} SDA={}",
                    (idle>>2)&1, (idle>>3)&1);

                // Helper: transmit one byte MSB-first, return ACK bit (0=ACK, 1=NACK).
                macro_rules! bb_tx {
                    ($byte:expr) => {{
                        let mut b: u8 = $byte;
                        for _ in 0..8u32 {
                            if (b & 0x80) != 0 { sda_rel!(); } else { sda_lo!(); }
                            scl_hi!();
                            scl_lo!();
                            b = b.wrapping_shl(1);
                        }
                        sda_rel!();          // release SDA for ACK
                        half_bit!();
                        scl_hi!();
                        let ack = sda_read!(); // 0=ACK, 1=NACK
                        scl_lo!();
                        ack
                    }};
                }

                // Helper: receive one byte MSB-first. master_ack=0 → send ACK, 1 → send NACK.
                macro_rules! bb_rx {
                    ($master_ack:expr) => {{
                        let mut byte: u8 = 0;
                        sda_rel!();
                        for _ in 0..8u32 {
                            scl_hi!();
                            byte = (byte << 1) | sda_read!() as u8;
                            scl_lo!();
                        }
                        // master ACK/NACK
                        if $master_ack == 0u32 { sda_lo!(); } else { sda_rel!(); }
                        half_bit!();
                        scl_hi!();
                        scl_lo!();
                        sda_rel!();
                        byte
                    }};
                }

                macro_rules! bb_start { () => {
                    sda_rel!(); scl_hi!(); half_bit!(); // idle
                    sda_lo!();  half_bit!();            // SDA LOW while SCL HIGH = START
                    scl_lo!();
                } }
                macro_rules! bb_stop { () => {
                    sda_lo!(); half_bit!();  // SDA LOW while SCL LOW
                    scl_hi!(); half_bit!();  // SCL HIGH
                    sda_rel!(); half_bit!(); // SDA HIGH while SCL HIGH = STOP
                } }

                {
                    // Before START: sample SDA one more time to confirm bus is still idle
                    let pre_sda = sda_read!();
                    bb_start!();
                    let ack_addr = bb_tx!(0x19u8 << 1); // addr+W = 0x32
                    // SDA after address ACK slot — confirm bus state
                    let post_addr_sda = sda_read!();
                    let ack_reg  = bb_tx!(0x0Fu8);      // WHO_AM_I register
                    // Repeated START
                    sda_rel!(); scl_hi!(); half_bit!();  // prep for rep-start
                    sda_lo!(); half_bit!(); scl_lo!();   // rep-START
                    let ack_r    = bb_tx!((0x19u8 << 1) | 1); // addr+R = 0x33
                    // Manual bit-by-bit read with SDA logging — replaces bb_rx! for diagnostic
                    sda_rel!();
                    let mut raw_bits = 0u8;
                    let mut sda_trace = 0u8; // bitmask of SDA state BEFORE scl_hi each bit
                    for bit in 0..8u32 {
                        // Read SDA before raising SCL (gives us slave's output during SCL LOW)
                        let pre_bit = sda_read!();
                        scl_hi!();
                        let on_bit = sda_read!(); // slave holds SDA valid during SCL HIGH
                        scl_lo!();
                        if on_bit == 1 { raw_bits |= 1 << (7 - bit as u8); }
                        if pre_bit == 1 { sda_trace |= 1 << (7 - bit as u8); }
                    }
                    // Master NACK
                    sda_rel!(); half_bit!();
                    scl_hi!(); scl_lo!(); sda_rel!();
                    bb_stop!();
                    info!("LIS2DTW12 WHO_AM_I: addr_ack={} post_addr_sda={} reg_ack={} read_ack={}",
                        ack_addr, post_addr_sda, ack_reg, ack_r);
                    info!("  pre_sda={} raw=0x{:02x}(SCL_hi) pre_scl=0x{:02x}(SCL_lo) expect 0x44",
                        pre_sda, raw_bits, sda_trace);
                    info!("  bit-by-bit (SCL_hi): b7={} b6={} b5={} b4={} b3={} b2={} b1={} b0={}",
                        (raw_bits>>7)&1,(raw_bits>>6)&1,(raw_bits>>5)&1,(raw_bits>>4)&1,
                        (raw_bits>>3)&1,(raw_bits>>2)&1,(raw_bits>>1)&1,raw_bits&1);
                }

                // ── TMAG5273 @ 0x22: read reg 0x00 (DEVICE_CONFIG_1 = typically 0x00 default) ──
                // and reg 0x0C (DEVICE_ID, expect 0x49) and 0x0D (MANUFACTURER_ID_LSB, expect 0x54)
                {
                    bb_start!();
                    let ack_a1 = bb_tx!(0x22u8 << 1);   // addr+W = 0x44
                    let ack_r1 = bb_tx!(0x00u8);         // start at reg 0x00
                    // Read 14 registers 0x00..0x0D in one burst (auto-increment)
                    sda_rel!(); scl_hi!(); half_bit!();
                    sda_lo!(); half_bit!(); scl_lo!();   // rep-START
                    let ack_r  = bb_tx!((0x22u8 << 1) | 1); // addr+R = 0x45
                    let mut tmag_regs = [0u8; 14];
                    for i in 0..14usize {
                        let master_ack = if i < 13 { 0u32 } else { 1u32 };
                        tmag_regs[i] = bb_rx!(master_ack);
                    }
                    bb_stop!();
                    info!("TMAG5273 addr_ack={} reg_ack={} rd_ack={}", ack_a1, ack_r1, ack_r);
                    for i in 0..14usize {
                        info!("  TMAG reg[0x{:02x}] = 0x{:02x}", i, tmag_regs[i]);
                    }
                    // reg[0x0C]=DEVICE_ID expect 0x49, reg[0x0D]=MANUFACTURER_ID_L expect 0x54
                    info!("  DEVICE_ID(0x0C)=0x{:02x}(expect 0x49) MANF_ID_L(0x0D)=0x{:02x}(expect 0x54)",
                        tmag_regs[0x0C], tmag_regs[0x0D]);
                }

                // ── SHT40 @ 0x44: soft-reset (0x94), then serial number (0x89) ──
                {
                    // Soft reset command: no read needed, just write
                    bb_start!();
                    let ack_rst_a = bb_tx!(0x44u8 << 1);  // addr+W = 0x88
                    let ack_rst_c = bb_tx!(0x94u8);        // soft reset command
                    bb_stop!();
                    info!("SHT40 soft-reset: addr_ack={} cmd_ack={}", ack_rst_a, ack_rst_c);
                    // Wait ≥1ms for reset
                    for _ in 0..100_000u32 { cortex_m::asm::nop(); } // ~1.5ms at 64MHz

                    // Read serial number: write 0x89, wait, read 6 bytes
                    bb_start!();
                    let ack_sn_a = bb_tx!(0x44u8 << 1);  // addr+W
                    let ack_sn_c = bb_tx!(0x89u8);        // serial number command
                    bb_stop!();
                    for _ in 0..100_000u32 { cortex_m::asm::nop(); } // wait for result
                    bb_start!();
                    let ack_sn_r = bb_tx!((0x44u8 << 1) | 1); // addr+R
                    let sn0 = bb_rx!(0u32); let sn1 = bb_rx!(0u32); let crc0 = bb_rx!(0u32);
                    let sn2 = bb_rx!(0u32); let sn3 = bb_rx!(0u32); let crc1 = bb_rx!(1u32);
                    bb_stop!();
                    info!("SHT40 serial: addr_ack={} cmd_ack={} rd_ack={}",
                        ack_sn_a, ack_sn_c, ack_sn_r);
                    info!("  SN=[{:02x} {:02x} {:02x} {:02x}] CRC=[{:02x} {:02x}]",
                        sn0, sn1, sn2, sn3, crc0, crc1);
                }

                // Final state
                let pin_end = rd(P1, 0x0c);
                info!("bb end: SCL={} SDA={}", (pin_end>>2)&1, (pin_end>>3)&1);
            }
            // Restore open-drain for TWIM probes that follow
            wr(P1, 0x80+2*4, 0x0000_0801);
            wr(P1, 0x80+3*4, 0x0000_0801);
            wr(P1, 0x04, (1<<2)|(1<<3));

            // ── CLEAN BUS PROBES (BEFORE S0-S5 which may trigger errata[105]) ──────────
            // State: TWIM DISABLED, PSEL disconnected, lines open-drain, SCL=1 SDA=1.
            // nRF54 TWIM: ENABLE puts peripheral in SUSPENDED. TASKS_RESUME (0x010) moves it to
            // RUNNING. DMA.TX.START while SUSPENDED drives SCL LOW (TWIM holds bus). MUST resume first.
            // Safe DISABLE: disconnect PSEL first when SCL=0 to reduce errata[105] risk.
            macro_rules! probe_clean {
                ($addr:expr, $label:literal) => {{
                    wr(TWIM, 0x600, 0x0000_0022);  // PSEL.SCL = P1.02 (set while DISABLED)
                    wr(TWIM, 0x604, 0x0000_0023);  // PSEL.SDA = P1.03
                    wr(TWIM, 0x524, 0x0198_0000);  // FREQUENCY = 100 kHz
                    wr(TWIM, 0x588, $addr as u32); // ADDRESS
                    wr(TWIM, 0x500, 6);            // ENABLE
                    for off in [0x104usize, 0x114, 0x128, 0x134, 0x138, 0x16c, 0x170] {
                        wr(TWIM, off, 0);           // clear events before RESUME
                    }
                    wr(TWIM, 0x010, 1);            // TASKS_RESUME: SUSPENDED → RUNNING
                    wr(TWIM, 0x73c, core::ptr::addr_of_mut!(TX_BUF) as u32);
                    wr(TWIM, 0x740, 1);            // DMA.TX.MAXCNT = 1
                    wr(TWIM, 0x200, 1 << 9);       // SHORT: LASTTX→STOP
                    wr(TWIM, 0x050, 1); // DMA.TX.START
                    let mut ps=0u32; let mut pe=0u32; let mut pts=0u32;
                    for _ in 0..3_000_000u32 {
                        ps  = rd(TWIM, 0x104);
                        pe  = rd(TWIM, 0x114);
                        pts = rd(TWIM, 0x16c);
                        if ps!=0 || pe!=0 { break; }
                        cortex_m::asm::nop();
                    }
                    let esrc = rd(TWIM, 0x4c4);
                    let txa  = rd(TWIM, 0x744);
                    let pin  = rd(P1, 0x0c);
                    let scl  = (pin >> 2) & 1;
                    info!("probe {}: stopped={} error={} TXSTARTED={} errorsrc=0x{:08x} tx.amt={} SCL={} SDA={}",
                        $label, ps, pe, pts, esrc, txa, scl, (pin >> 3) & 1);
                    if scl == 0 {
                        wr(TWIM, 0x600, 0x8000_0000);
                        wr(TWIM, 0x604, 0x8000_0000);
                        info!("  [{}] SCL=0 before DISABLE — psel disconnect first", $label);
                    }
                    wr(TWIM, 0x500, 0); // DISABLE
                    wr(TWIM, 0x600, 0x8000_0000);
                    wr(TWIM, 0x604, 0x8000_0000);
                    if scl == 0 {
                        wr(P1, 0x04, (1<<2)|(1<<3));
                        wr(P1, 0x80+2*4, 0x0000_0001);
                        wr(P1, 0x80+3*4, 0x0000_0000);
                        for _ in 0..9u32 {
                            wr(P1, 0x08, 1<<2); for _ in 0..100u32 { cortex_m::asm::nop(); }
                            wr(P1, 0x04, 1<<2); for _ in 0..100u32 { cortex_m::asm::nop(); }
                        }
                        wr(P1, 0x80+3*4, 0x0000_0001);
                        wr(P1, 0x08, 1<<3); for _ in 0..100u32 { cortex_m::asm::nop(); }
                        wr(P1, 0x04, 1<<3); for _ in 0..500u32 { cortex_m::asm::nop(); }
                        let pin_r = rd(P1, 0x0c);
                        info!("  [{}] GPIO recovery: SCL={} SDA={}", $label, (pin_r>>2)&1, (pin_r>>3)&1);
                    }
                    wr(P1, 0x80+2*4, 0x0000_0801);
                    wr(P1, 0x80+3*4, 0x0000_0801);
                }};
            }
            probe_clean!(0x7Fu8,  "0x7F(no-slave)");  // Must ANACK: proves TWIM detects NACK
            probe_clean!(0x19u8,  "0x19(LIS2DTW12)"); // Accelerometer
            probe_clean!(0x44u8,  "0x44(SHT40)");     // Humidity sensor

            // ── TWIM ENABLE TIMING PROBE ──────────────────────────────────────
            // Pinpoint exactly when/why lines go LOW. Read P1_IN after each step.

            // Step 0: Check lines are HIGH before touching TWIM (PSEL still disconnected)
            let in_before_psel = rd(P1, 0x0c);
            info!("S0 before PSEL set:   SCL_in={} SDA_in={}",
                (in_before_psel >> 2) & 1, (in_before_psel >> 3) & 1);

            // Step 1: Set PSEL (TWIM still DISABLED — should NOT change GPIO)
            wr(TWIM, 0x600, 0x0000_0022); // PSEL.SCL = P1.02
            wr(TWIM, 0x604, 0x0000_0023); // PSEL.SDA = P1.03
            cortex_m::asm::dsb();
            let in_after_psel = rd(P1, 0x0c);
            info!("S1 after PSEL (dis):  SCL_in={} SDA_in={}",
                (in_after_psel >> 2) & 1, (in_after_psel >> 3) & 1);

            // Step 2: ENABLE — does this change line state?
            wr(TWIM, 0x524, 0x0198_0000); // FREQUENCY = 100 kHz
            wr(TWIM, 0x588, 0x0000_0022); // ADDRESS = 0x22
            wr(TWIM, 0x500, 6); // ENABLE
            cortex_m::asm::dsb();
            let in_after_enable = rd(P1, 0x0c);
            info!("S2 after ENABLE:      SCL_in={} SDA_in={}",
                (in_after_enable >> 2) & 1, (in_after_enable >> 3) & 1);

            // Step 3: No RESUME for fresh ENABLE (only needed from SUSPENDED state).

            let tx_ptr = core::ptr::addr_of_mut!(TX_BUF) as u32;

            // Step 4: Set up DMA and SHORTS
            wr(TWIM, 0x73c, tx_ptr); // DMA.TX.PTR
            wr(TWIM, 0x740, 1);      // DMA.TX.MAXCNT = 1 byte
            wr(TWIM, 0x200, 1 << 9); // SHORT: LASTTX→STOP only
            for off in [0x104, 0x114, 0x128, 0x134, 0x138, 0x14c, 0x150, 0x154, 0x168, 0x16c, 0x170] {
                wr(TWIM, off, 0);
            }
            cortex_m::asm::dsb();
            let in_after_dma = rd(P1, 0x0c);
            info!("S4 after DMA setup:   SCL_in={} SDA_in={}",
                (in_after_dma >> 2) & 1, (in_after_dma >> 3) & 1);

            // Step 5: DMA.TX.START
            wr(TWIM, 0x050, 1); // tasks_dma.tx.start
            cortex_m::asm::dsb();
            let in_after_start = rd(P1, 0x0c);
            info!("S5 after TX.START:    SCL_in={} SDA_in={}",
                (in_after_start >> 2) & 1, (in_after_start >> 3) & 1);

            // Poll for ~500ms max; log when SCL is released so we know if slave un-stretches.
            let mut stopped=0u32; let mut error=0u32; let mut suspended=0u32;
            let mut txstarted=0u32; let mut lasttx=0u32;
            let mut txbuserr=0u32;
            // SCL drop/rise tracking. Each iteration ≈ 7 MMIO reads × ~300ns = ~2.2µs.
            // At 100kHz I2C: 1 bit = 10µs ≈ 4.5 iters. 9 bits (addr+ACK) ≈ 40 iters.
            // first_drop shortly after start = clock during normal bit clocking.
            // first_drop >> 40 = clock-stretch AFTER address phase.
            // last_rise = last iter SCL was observed HIGH (None = never observed HIGH).
            let mut scl_first_drop: Option<u32> = None;
            let mut scl_last_rise:  Option<u32> = None;
            for i in 0..5_000_000u32 {  // ~11s max — enough for 10 I2C timeouts
                stopped   = rd(TWIM, 0x104);
                error     = rd(TWIM, 0x114);
                suspended = rd(TWIM, 0x128);
                lasttx    = rd(TWIM, 0x138);
                txstarted = rd(TWIM, 0x16c);
                txbuserr  = rd(TWIM, 0x170);
                let scl = (rd(P1, 0x0c) >> 2) & 1;
                if scl == 0 && scl_first_drop.is_none() { scl_first_drop = Some(i); }
                if scl == 1 { scl_last_rise = Some(i); }
                if stopped!=0 || error!=0 || suspended!=0 || lasttx!=0 || txbuserr!=0 { break; }
                cortex_m::asm::nop();
            }
            let errorsrc  = rd(TWIM, 0x4c4);
            let tx_amount = rd(TWIM, 0x744);
            let in_final  = rd(P1, 0x0c);
            // If timed out with SCL=0, send TASKS_STOP to try to abort transaction.
            // This generates a STOP on the bus even with slave clock-stretching.
            // (Pushes TWIM to generate STOP; may or may not succeed depending on stretch)
            if stopped == 0 && error == 0 {
                info!("  addr=0x22: TIMEOUT — sending TASKS_STOP");
                wr(TWIM, 0x014, 1); // TASKS_STOP
                let mut stop_wait = 500_000u32;
                while stop_wait > 0 && rd(TWIM, 0x104) == 0 { stop_wait -= 1; cortex_m::asm::nop(); }
                let stopped_after = rd(TWIM, 0x104);
                let pin_after_stop = rd(P1, 0x0c);
                info!("  after TASKS_STOP: stopped={} SCL={} SDA={}",
                    stopped_after, (pin_after_stop>>2)&1, (pin_after_stop>>3)&1);
            }
            info!("addr=0x22: stopped={} error={} lasttx={} TXSTARTED={} errorsrc=0x{:08x} tx.amount={} SCL={} SDA={}",
                stopped, error, lasttx, txstarted, errorsrc, tx_amount,
                (in_final >> 2) & 1, (in_final >> 3) & 1);
            info!("  SCL first_drop={:?}  last_rise={:?}  (≈4.5iters/bit  9bits≈40iters)",
                scl_first_drop, scl_last_rise);

            // ── FINAL GPIO BUS RECOVERY (no more TWIM after this) ──────────────
            // TWIM may have errata[105]. Disconnect PSEL before DISABLE when SCL=0.
            let fin_scl = (in_final >> 2) & 1;
            if fin_scl == 0 {
                wr(TWIM, 0x600, 0x8000_0000);
                wr(TWIM, 0x604, 0x8000_0000);
            }
            wr(TWIM, 0x500, 0); // DISABLE
            wr(TWIM, 0x600, 0x8000_0000);
            wr(TWIM, 0x604, 0x8000_0000);
            wr(P1, 0x04, (1<<2)|(1<<3));
            wr(P1, 0x80+2*4, 0x0000_0001);
            wr(P1, 0x80+3*4, 0x0000_0000);
            for _ in 0..9u32 {
                wr(P1, 0x08, 1<<2); for _ in 0..100u32 { cortex_m::asm::nop(); }
                wr(P1, 0x04, 1<<2); for _ in 0..100u32 { cortex_m::asm::nop(); }
            }
            wr(P1, 0x80+3*4, 0x0000_0001);
            wr(P1, 0x08, 1<<3); for _ in 0..100u32 { cortex_m::asm::nop(); }
            wr(P1, 0x04, 1<<3); for _ in 0..500u32 { cortex_m::asm::nop(); }
            wr(P1, 0x80+2*4, 0x0000_0801);
            wr(P1, 0x80+3*4, 0x0000_0801);
            let pin_final = rd(P1, 0x0c);
            info!("Final GPIO recovery: SCL={} SDA={}", (pin_final>>2)&1, (pin_final>>3)&1);
        }
    }

    let mut buf = [0u8; 1];
    info!("embassy write_read ...");
    match i2c.write_read(0x22u8, &[0x0Du8], &mut buf).await {
        Ok(()) => info!("OK: 0x{:02x}", buf[0]),
        Err(e) => error!("FAIL: {:?}", defmt::Debug2Format(&e)),
    }
    info!("done");
    loop { embassy_time::Timer::after_secs(60).await; }
}
