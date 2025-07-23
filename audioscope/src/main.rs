#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use common::consts::*;
use core::u32;
use defmt::info;
use defmt_rtt as _;
use fugit::HertzU32;
use panic_probe as _;
use rp2040_cycle_counter::Rp2040CycleCounter;
use rp2040_hal::{
    clocks::{Clock, ClockSource, ClocksManager, InitError},
    pac::{self},
    pll::{common_configs::PLL_USB_48MHZ, setup_pll_blocking},
    watchdog::Watchdog,
    xosc::setup_xosc_blocking,
};

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    watchdog.enable_tick_generation((EXTERNAL_XTAL_FREQ_HZ.raw() / 1_000_000) as u8);

    let mut clocks = ClocksManager::new(pac.CLOCKS);

    let xosc = setup_xosc_blocking(pac.XOSC, EXTERNAL_XTAL_FREQ_HZ)
        .map_err(InitError::XoscErr)
        .ok()
        .unwrap();

    {
        let pll_sys = setup_pll_blocking(
            pac.PLL_SYS,
            xosc.operating_frequency(),
            common::plls::SYS_PLL_CONFIG_153P6MHZ,
            &mut clocks,
            &mut pac.RESETS,
        )
        .map_err(InitError::PllError)
        .ok()
        .unwrap();

        let pll_usb = setup_pll_blocking(
            pac.PLL_USB,
            xosc.operating_frequency(),
            PLL_USB_48MHZ,
            &mut clocks,
            &mut pac.RESETS,
        )
        .map_err(InitError::PllError)
        .ok()
        .unwrap();

        clocks
            .reference_clock
            .configure_clock(&xosc, xosc.get_freq())
            .map_err(InitError::ClockError)
            .ok()
            .unwrap();

        clocks
            .system_clock
            .configure_clock(&pll_sys, pll_sys.get_freq())
            .map_err(InitError::ClockError)
            .ok()
            .unwrap();

        clocks
            .usb_clock
            .configure_clock(&pll_usb, pll_usb.get_freq())
            .map_err(InitError::ClockError)
            .ok()
            .unwrap();

        clocks
            .adc_clock
            .configure_clock(&pll_usb, pll_usb.get_freq())
            .map_err(InitError::ClockError)
            .ok()
            .unwrap();

        clocks
            .rtc_clock
            .configure_clock(&pll_usb, HertzU32::from_raw(46875u32))
            .map_err(InitError::ClockError)
            .ok()
            .unwrap();

        clocks
            .peripheral_clock
            .configure_clock(&clocks.system_clock, clocks.system_clock.freq())
            .map_err(InitError::ClockError)
            .ok()
            .unwrap();
    }

    let mut benchmark = Rp2040CycleCounter::new(core.SYST);

    benchmark.setup();

    let mut cycles_sum = 0u64;
    let mut runs_count = 0u32;

    loop {
        benchmark.start();
        // TODO: benchmark something if necessary
        let cycles = benchmark.end().unwrap();
        let (new_cycles_sum, overflowed) = cycles_sum.overflowing_add(cycles as u64);
        cycles_sum = new_cycles_sum;

        if overflowed {
            info!("Can't fit more run results in u64. Stalling.");
            loop {}
        }

        runs_count += 1;

        info!(
            "cycles: {}, runs: {}, avg cycles: {}",
            cycles,
            runs_count,
            cycles_sum as f64 / runs_count as f64
        );
    }
}
