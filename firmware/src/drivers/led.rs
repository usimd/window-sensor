//! LED driver — minimal abstraction for D1 (Blue) and D2 (Red)

use embassy_nrf::gpio::Output;
use embassy_time::Timer;

pub struct Led<'d> {
    pin: Output<'d>,
}

impl<'d> Led<'d> {
    pub fn new(pin: Output<'d>) -> Self {
        Self { pin }
    }

    pub fn on(&mut self) {
        self.pin.set_high();
    }

    pub fn off(&mut self) {
        self.pin.set_low();
    }

    pub async fn flash_ms(&mut self, ms: u32) {
        self.on();
        Timer::after_millis(ms as u64).await;
        self.off();
    }

    pub async fn blink(&mut self, on_ms: u32, off_ms: u32, count: u32) {
        for _ in 0..count {
            self.flash_ms(on_ms).await;
            Timer::after_millis(off_ms as u64).await;
        }
    }
}
