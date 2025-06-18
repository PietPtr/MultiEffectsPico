#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use common::consts::*;
use cortex_m::singleton;
use defmt::*;
use defmt_rtt as _;
use fixed::types::{I1F15, U8F8};
use fugit::HertzU32;
use panic_probe as _;
use rp2040_hal::{
    clocks::{Clock, ClockSource, ClocksManager, InitError},
    dma::{double_buffer, DMAExt},
    gpio::{self},
    multicore::{Multicore, Stack},
    pac,
    pio::{PIOExt, PinDir},
    pll::{common_configs::PLL_USB_48MHZ, setup_pll_blocking},
    sio::Sio,
    watchdog::Watchdog,
    xosc::setup_xosc_blocking,
};

use rytmos_synth::effect::{
    amplify::{Amplify, AmplifySettings},
    Effect,
};

static mut CORE1_STACK: Stack<4096> = Stack::new();

pub const BUFFER_SIZE: usize = 16;

#[allow(dead_code)]
fn setup_dac_triangle_wave(sys_freq: HertzU32) -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };
    let sio = Sio::new(pac.SIO);
    let pins = gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let (mut pio0, sm0, _, _, sm3) = pac.PIO0.split(&mut pac.RESETS);

    let dac_output = rp2040_i2s::I2SOutput::new(
        &mut pio0,
        rp2040_i2s::PioClockDivider::FromSystemClock(sys_freq),
        sm0,
        pins.gpio6,
        pins.gpio7,
        pins.gpio8,
    )
    .unwrap();

    let (sm0, _, dac_fifo_tx) = dac_output.split();

    let dma_channels = pac.DMA.split(&mut pac.RESETS);

    let i2s_tx_buf1 = singleton!(: [u32; BUFFER_SIZE*2] = [0; BUFFER_SIZE*2]).unwrap();
    let i2s_tx_buf2 = singleton!(: [u32; BUFFER_SIZE*2] = [0; BUFFER_SIZE*2]).unwrap();
    let i2s_dma_config = double_buffer::Config::new(
        (dma_channels.ch0, dma_channels.ch1),
        i2s_tx_buf1,
        dac_fifo_tx,
    );
    let i2s_tx_transfer = i2s_dma_config.start();
    let mut i2s_tx_transfer = i2s_tx_transfer.read_next(i2s_tx_buf2);

    // -- move to lib --

    let sck_pin: gpio::Pin<gpio::bank0::Gpio9, gpio::FunctionPio0, gpio::PullDown> =
        pins.gpio9.reconfigure();
    {
        #[rustfmt::skip]
        let sck_pio_program = pio_proc::pio_asm!(
            ".wrap_target",
            "    set pins, 0b1",
            "    set pins, 0b0",
            ".wrap",
        );

        let installed = pio0.install(&sck_pio_program.program).unwrap();

        let sck_clock_divisor: u16 = 6; // 256 times the lr clock => 48k => 12.288MHz => lib found clock / 2

        let (mut sck_sm, _, _) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
            .set_pins(sck_pin.id().num, 1)
            .clock_divisor_fixed_point(sck_clock_divisor, 64)
            .build(sm3);

        sck_sm.set_pindirs([(sck_pin.id().num, PinDir::Output)]);
        sck_sm.start();
        info!("sck state machine started");
    }

    // -- ^^^^^^^^^^^ --

    sm0.start();

    const WAVELENGTH: i32 = 500;

    let mut sample: u32 = 0;

    loop {
        let (next_tx_buf, next_tx_transfer) = i2s_tx_transfer.wait();

        for e in next_tx_buf.iter_mut() {
            (sample, _) = sample.overflowing_add(7000000);
            *e = sample;
        }

        i2s_tx_transfer = next_tx_transfer.read_next(next_tx_buf);
    }
}

#[allow(dead_code)]
fn setup_adc_and_dac(sys_freq: HertzU32) -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };
    let sio = Sio::new(pac.SIO);
    let pins = gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let (mut pio0, sm0, sm1, _, sm3) = pac.PIO0.split(&mut pac.RESETS);

    // -- move to lib --

    let sck_pin: gpio::Pin<gpio::bank0::Gpio9, gpio::FunctionPio0, gpio::PullDown> =
        pins.gpio9.reconfigure();
    {
        #[rustfmt::skip]
        let sck_pio_program = pio_proc::pio_asm!(
            ".wrap_target",
            "    set pins, 0b1",
            "    set pins, 0b0",
            ".wrap",
        );

        let installed = pio0.install(&sck_pio_program.program).unwrap();

        let sck_clock_divisor: u16 = 6; // 256 times the lr clock => 48k => 12.288MHz => lib found clock / 2

        let (mut sck_sm, _, _) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
            .set_pins(sck_pin.id().num, 1)
            .clock_divisor_fixed_point(sck_clock_divisor, 64)
            .build(sm3);

        sck_sm.set_pindirs([(sck_pin.id().num, PinDir::Output)]);
        sck_sm.start();
        info!("sck state machine started");
    }

    // -- ^^^^^^^^^^^ --

    let dac_output = rp2040_i2s::I2SOutput::new(
        &mut pio0,
        rp2040_i2s::PioClockDivider::FromSystemClock(sys_freq),
        sm0,
        pins.gpio6,
        pins.gpio7,
        pins.gpio8,
    )
    .unwrap();

    let adc_input = rp2040_i2s::I2SInput::new(
        &mut pio0,
        rp2040_i2s::PioClockDivider::FromSystemClock(sys_freq),
        sm1,
        pins.gpio10,
        pins.gpio11,
        pins.gpio12,
    )
    .unwrap();

    let (dac_sm, _, dac_fifo_tx) = dac_output.split();
    let (adc_sm, adc_fifo_rx, _) = adc_input.split();

    dac_sm.start();
    adc_sm.start();

    let dma_channels = pac.DMA.split(&mut pac.RESETS);

    let i2s_tx_buf1 = singleton!(: [u32; BUFFER_SIZE*2] = [0; BUFFER_SIZE*2]).unwrap();
    let i2s_tx_buf2 = singleton!(: [u32; BUFFER_SIZE*2] = [0; BUFFER_SIZE*2]).unwrap();
    let i2s_dma_config = double_buffer::Config::new(
        (dma_channels.ch0, dma_channels.ch1),
        i2s_tx_buf1,
        dac_fifo_tx,
    );
    let i2s_tx_transfer = i2s_dma_config.start();
    let mut i2s_tx_transfer = i2s_tx_transfer.read_next(i2s_tx_buf2);

    let i2s_rx_buf1 = singleton!(: [u32; BUFFER_SIZE*2] = [0; BUFFER_SIZE*2]).unwrap();
    let i2s_rx_buf2 = singleton!(: [u32; BUFFER_SIZE*2] = [0; BUFFER_SIZE*2]).unwrap();
    let i2s_dma_config = double_buffer::Config::new(
        (dma_channels.ch2, dma_channels.ch3),
        adc_fifo_rx,
        i2s_rx_buf1,
    );
    let i2s_rx_transfer = i2s_dma_config.start();
    let mut i2s_rx_transfer = i2s_rx_transfer.write_next(i2s_rx_buf2);

    let mut effect = Amplify::make(
        0,
        AmplifySettings {
            // amplification: U8F8::from_num(U8F8::MAX / 8),
            amplification: U8F8::from_num(8),
        },
    );
    let mut sample: I1F15;

    loop {
        let (next_rx_buf, next_rx_transfer) = i2s_rx_transfer.wait();

        // apply effects
        for (_i, rx_sample) in next_rx_buf.iter_mut().enumerate() {
            // Somehow the first bit is not set, probably som glitch in the setup
            // This shift however makes sure negative numbers are negative
            *rx_sample <<= 1;
            let fixed_sample = I1F15::from_bits((*rx_sample >> 16) as i16);

            // Here custom effect code can be inserted
            sample = effect.next(fixed_sample);

            // and here it's converted back to something working
            *rx_sample = (sample.to_bits() as u32) << 16
        }

        let (next_tx_buf, next_tx_transfer) = i2s_tx_transfer.wait();

        // write computed samples into tx
        for (&rx_sample, tx_sample) in next_rx_buf.iter().zip(next_tx_buf.iter_mut()) {
            *tx_sample = rx_sample
        }

        i2s_tx_transfer = next_tx_transfer.read_next(next_tx_buf);
        i2s_rx_transfer = next_rx_transfer.write_next(next_rx_buf);
    }
}

#[allow(dead_code)]
fn setup_dual_adc_and_dac(sys_freq: HertzU32) -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };
    let sio = Sio::new(pac.SIO);
    let pins = gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let (mut pio0, sm0, sm1, sm2, sm3) = pac.PIO0.split(&mut pac.RESETS);

    // -- move to lib --

    let sck_pin: gpio::Pin<gpio::bank0::Gpio9, gpio::FunctionPio0, gpio::PullDown> =
        pins.gpio9.reconfigure();
    {
        #[rustfmt::skip]
        let sck_pio_program = pio_proc::pio_asm!(
            ".wrap_target",
            "    set pins, 0b1",
            "    set pins, 0b0",
            ".wrap",
        );

        let installed = pio0.install(&sck_pio_program.program).unwrap();

        let sck_clock_divisor: u16 = 6; // 256 times the lr clock => 48k => 12.288MHz => lib found clock / 2

        let (mut sck_sm, _, _) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
            .set_pins(sck_pin.id().num, 1)
            .clock_divisor_fixed_point(sck_clock_divisor, 64)
            .build(sm3);

        sck_sm.set_pindirs([(sck_pin.id().num, PinDir::Output)]);
        sck_sm.start();
        info!("sck state machine started");
    }

    // -- ^^^^^^^^^^^ --

    let dac_output = rp2040_i2s::I2SOutput::new(
        &mut pio0,
        rp2040_i2s::PioClockDivider::FromSystemClock(sys_freq),
        sm0,
        pins.gpio6,
        pins.gpio7,
        pins.gpio8,
    )
    .unwrap();

    let adc_aux_input = rp2040_i2s::I2SInput::new(
        &mut pio0,
        rp2040_i2s::PioClockDivider::FromSystemClock(sys_freq),
        sm1,
        pins.gpio10,
        pins.gpio11,
        pins.gpio12,
    )
    .unwrap();

    let adc_jack_input = rp2040_i2s::I2SInput::new(
        &mut pio0,
        rp2040_i2s::PioClockDivider::FromSystemClock(sys_freq),
        sm2,
        pins.gpio13,
        pins.gpio14,
        pins.gpio15,
    )
    .unwrap();

    let (dac_sm, _, dac_fifo_tx) = dac_output.split();
    let (adc_aux_sm, adc_aux_fifo_rx, _) = adc_aux_input.split();
    let (adc_jack_sm, adc_jack_fifo_rx, _) = adc_jack_input.split();

    dac_sm.start();
    adc_aux_sm.start();
    adc_jack_sm.start();

    let dma_channels = pac.DMA.split(&mut pac.RESETS);

    let i2s_tx_buf1 = singleton!(: [u32; BUFFER_SIZE*2] = [0; BUFFER_SIZE*2]).unwrap();
    let i2s_tx_buf2 = singleton!(: [u32; BUFFER_SIZE*2] = [0; BUFFER_SIZE*2]).unwrap();
    let i2s_dma_config = double_buffer::Config::new(
        (dma_channels.ch0, dma_channels.ch1),
        i2s_tx_buf1,
        dac_fifo_tx,
    );
    let i2s_tx_transfer = i2s_dma_config.start();
    let mut i2s_tx_transfer = i2s_tx_transfer.read_next(i2s_tx_buf2);

    let i2s_rx_buf1 = singleton!(: [u32; BUFFER_SIZE*2] = [0; BUFFER_SIZE*2]).unwrap();
    let i2s_rx_buf2 = singleton!(: [u32; BUFFER_SIZE*2] = [0; BUFFER_SIZE*2]).unwrap();
    let i2s_dma_config = double_buffer::Config::new(
        (dma_channels.ch2, dma_channels.ch3),
        adc_aux_fifo_rx,
        i2s_rx_buf1,
    );
    let aux_rx_transfer = i2s_dma_config.start();
    let mut aux_rx_transfer = aux_rx_transfer.write_next(i2s_rx_buf2);

    let i2s_rx_buf1 = singleton!(: [u32; BUFFER_SIZE*2] = [0; BUFFER_SIZE*2]).unwrap();
    let i2s_rx_buf2 = singleton!(: [u32; BUFFER_SIZE*2] = [0; BUFFER_SIZE*2]).unwrap();
    let i2s_dma_config = double_buffer::Config::new(
        (dma_channels.ch4, dma_channels.ch5),
        adc_jack_fifo_rx,
        i2s_rx_buf1,
    );
    let jack_rx_transfer = i2s_dma_config.start();
    let mut jack_rx_transfer = jack_rx_transfer.write_next(i2s_rx_buf2);

    loop {
        let (next_aux_rx_buf, next_aux_rx_transfer) = aux_rx_transfer.wait();
        let (next_jack_rx_buf, next_jack_rx_transfer) = jack_rx_transfer.wait();

        // mix both inputs 50/50
        for (aux_sample, jack_sample) in next_aux_rx_buf.iter_mut().zip(next_jack_rx_buf.iter_mut())
        {
            // prep samples
            *aux_sample <<= 1;
            *jack_sample <<= 1;

            let signed_aux_sample = *aux_sample as i32;
            let signed_jack_sample = *jack_sample as i32;

            // mix / apply effects
            let new_sample = signed_aux_sample.saturating_add(signed_jack_sample);

            // store
            *aux_sample = new_sample as u32;
        }

        let (next_tx_buf, next_tx_transfer) = i2s_tx_transfer.wait();

        // write computed samples into tx
        for (&rx_sample, tx_sample) in next_aux_rx_buf.iter().zip(next_tx_buf.iter_mut()) {
            *tx_sample = rx_sample
        }

        i2s_tx_transfer = next_tx_transfer.read_next(next_tx_buf);
        jack_rx_transfer = next_jack_rx_transfer.write_next(next_jack_rx_buf);
        aux_rx_transfer = next_aux_rx_transfer.write_next(next_aux_rx_buf);
    }
}

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

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

    // Setup the other core
    let sys_freq = clocks.system_clock.freq().to_Hz();
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    #[allow(static_mut_refs)]
    let _ = core1.spawn(unsafe { CORE1_STACK.take().unwrap() }, move || {
        // setup_dac_triangle_wave(clocks.system_clock.freq())
        // setup_adc_and_dac(clocks.system_clock.freq())
        setup_dual_adc_and_dac(clocks.system_clock.freq())
    });

    info!("Set up at sys_freq = {}Hz", sys_freq);
    info!("Start I/O thread.");

    loop {}
}
