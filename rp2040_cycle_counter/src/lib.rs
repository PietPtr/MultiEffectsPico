#![no_std]

use cortex_m::peripheral::syst::SystClkSource;
use rp2040_hal::pac::SYST;

pub struct Rp2040CycleCounter {
    systick: SYST,
    start_time: Option<u32>,
}

impl Rp2040CycleCounter {
    pub fn new(systick: SYST) -> Self {
        Self {
            systick,
            start_time: None,
        }
    }

    pub fn setup(&mut self) {
        self.systick.enable_counter();
        unsafe { self.systick.rvr.write(0x00FFFFFF) };
        self.systick.set_clock_source(SystClkSource::Core);
    }

    pub fn current_timer_value(&mut self) -> u32 {
        self.systick.cvr.read()
    }

    pub fn start(&mut self) {
        self.start_time = Some(self.current_timer_value())
    }

    pub fn end(&mut self) -> Result<u32, Rp2040CycleCounterError> {
        let end_time = self.current_timer_value();
        let has_wrapped = self.systick.has_wrapped();

        let Some(start_time) = self.start_time else {
            return Err(Rp2040CycleCounterError::NotStarted);
        };

        if has_wrapped {
            return Err(Rp2040CycleCounterError::Wrapped(start_time - end_time));
        }

        self.systick.clear_current();
        self.start_time = Some(end_time);

        // SysTick timer counts down, so subtraction is reversed.
        Ok(start_time - end_time)
    }

    pub fn free(self) -> SYST {
        self.systick
    }
}

#[derive(Debug)]
pub enum Rp2040CycleCounterError {
    NotStarted,
    /// Counter has wrapped since calling start(), but the difference is given anyway.
    Wrapped(u32),
}
