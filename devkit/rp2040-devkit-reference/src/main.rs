#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use common::consts::*;
use core::fmt::Write as _;
use core::{
    cell::RefCell,
    sync::atomic::{AtomicU32, Ordering},
    u32,
};
use cortex_m::{interrupt::Mutex, singleton};
use defmt::info;
use defmt_rtt as _;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::{adc::OneShot, blocking::i2c::Write, digital::v2::InputPin};
use fixed::types::{I1F15, U8F8};
use frequency_monitor::FrequencyMonitor;
use fugit::{Duration, HertzU32, RateExtU32};
use heapless::{String, Vec};
use mcp23017::MCP23017;
use panic_probe as _;
use rotary_encoder_embedded::{Direction, RotaryEncoder};
use rp2040_hal::{
    adc::AdcPin,
    clocks::{Clock, ClockSource, ClocksManager, InitError},
    dma::{double_buffer, DMAExt},
    gpio::{self, DynFunction, DynPinId, FunctionSioInput, Pin, PullDown, PullUp},
    multicore::{Multicore, Stack},
    pac::{self, interrupt, NVIC},
    pio::{PIOExt, PinDir},
    pll::{common_configs::PLL_USB_48MHZ, setup_pll_blocking},
    sio::Sio,
    timer::{Alarm, Alarm0},
    watchdog::Watchdog,
    xosc::setup_xosc_blocking,
    Adc, Timer, I2C,
};
use sh1106::mode::GraphicsMode;

use rytmos_synth::effect::{
    amplify::{Amplify, AmplifySettings},
    Effect,
};

static mut CORE1_STACK: Stack<4096> = Stack::new();

// TODO: moving 48k samples from one core to the other over shared memory should not be a hardware constraint.
// ring buffer maybe? ringbuffer crate.
static FFT_MON_TIME_DOMAIN_DATA_MUTEX: critical_section::Mutex<
    RefCell<[u32; FrequencyMonitor::FFT_SIZE]>,
> = critical_section::Mutex::new(RefCell::new([0; FrequencyMonitor::FFT_SIZE]));

pub const BUFFER_SIZE: usize = 16;

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

    // Effects
    let mut effect = Amplify::make(
        0,
        AmplifySettings {
            amplification: U8F8::from_num(U8F8::MAX / 8),
            // amplification: U8F8::from_num(1),
        },
    );
    let mut sample: I1F15;

    let mut fft_time_domain_store: Vec<u32, { FrequencyMonitor::FFT_SIZE }> = Vec::new();

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

            match fft_time_domain_store.push(*aux_sample) {
                Ok(()) => (),
                Err(_) => {
                    // local buffer full, try to obtain mutex, write it, and clear local buffer

                    critical_section::with(|cs| {
                        let Ok(mut t) = FFT_MON_TIME_DOMAIN_DATA_MUTEX.borrow(cs).try_borrow_mut()
                        else {
                            // couldn't borrow it now, so it's being read by the consuming thread,
                            // throw current data away and start over
                            fft_time_domain_store.clear();
                            return;
                        };

                        t.copy_from_slice(&fft_time_domain_store);
                        fft_time_domain_store.clear();
                    });
                }
            };
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

    let pins = gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Setup double buffer for FFT visualizer
    let dma_channels = pac.DMA.split(&mut pac.RESETS);

    // Setup the other core
    let sys_freq = clocks.system_clock.freq();
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    #[allow(static_mut_refs)]
    let _ = core1.spawn(unsafe { CORE1_STACK.take().unwrap() }, move || {
        setup_dual_adc_and_dac(sys_freq)
    });

    info!("Set up at sys_freq = {}Hz", sys_freq.to_Hz());
    info!("Start I/O thread.");

    let encoder1 = RotaryEncoder::new(
        pins.gpio17.into_pull_up_input().into_dyn_pin(),
        pins.gpio18.into_pull_up_input().into_dyn_pin(),
    )
    .into_standard_mode();
    let encoder2 = RotaryEncoder::new(
        pins.gpio20.into_pull_up_input().into_dyn_pin(),
        pins.gpio21.into_pull_up_input().into_dyn_pin(),
    )
    .into_standard_mode();
    let encoder3 = RotaryEncoder::new(
        pins.gpio4.into_pull_up_input().into_dyn_pin(),
        pins.gpio5.into_pull_up_input().into_dyn_pin(),
    )
    .into_standard_mode();

    let mut timer: Timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    NVIC::unpend(pac::Interrupt::TIMER_IRQ_0);
    unsafe {
        NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    let mut alarm = timer.alarm_0().unwrap();
    alarm
        .schedule(Duration::<u32, 1, 1000000>::millis(2))
        .unwrap();
    alarm.enable_interrupt();

    cortex_m::interrupt::free(|cs| {
        ALARM.borrow(cs).replace(Some(alarm));
        GLOBAL_ENCODERS
            .borrow(cs)
            .replace(Some([encoder1, encoder2, encoder3]))
    });

    let mut pot_readers: [AdcPin<Pin<DynPinId, DynFunction, PullDown>>; 3] = [
        AdcPin::new(pins.gpio26.reconfigure().into_dyn_pin()).unwrap(),
        AdcPin::new(pins.gpio27.reconfigure().into_dyn_pin()).unwrap(),
        AdcPin::new(pins.gpio28.reconfigure().into_dyn_pin()).unwrap(),
    ];

    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);

    let i2c = I2C::i2c0(
        pac.I2C0,
        pins.gpio0.reconfigure(),
        pins.gpio1.reconfigure(),
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let bus = shared_bus::BusManagerSimple::new(i2c);

    let mut io_expander = MCP23017::default(bus.acquire_i2c()).unwrap();

    io_expander.init_hardware().unwrap();
    io_expander.all_pin_mode(mcp23017::PinMode::OUTPUT).unwrap();

    // Couldn't get pindirs and pullups working with the library, so here's it done manually
    let mut test_i2c = bus.acquire_i2c();
    test_i2c
        .write(0x20, &[0x00, 0b1100_0000, 0b1100_0000])
        .unwrap();
    test_i2c
        .write(0x20, &[0x0c, 0b1100_0000, 0b1100_0000])
        .unwrap();

    let enc_switches: [Pin<DynPinId, FunctionSioInput, PullUp>; 3] = [
        pins.gpio16.reconfigure().into_dyn_pin(),
        pins.gpio19.reconfigure().into_dyn_pin(),
        pins.gpio3.reconfigure().into_dyn_pin(),
    ];

    let mut display: GraphicsMode<_> = sh1106::Builder::new()
        .with_rotation(sh1106::prelude::DisplayRotation::Rotate180)
        .connect_i2c(bus.acquire_i2c())
        .into();
    display.init().unwrap();
    display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let activities = [
        "    ", "     ", "|   ", "||   ", "|||  ", "|||| ", " ||||", "  |||", "   ||", "    |",
    ];
    let mut activity = 0;

    const BUTTON1_PIN: u8 = 6;
    const BUTTON2_PIN: u8 = 7;
    const BUTTON3_PIN: u8 = 14;
    const BUTTON4_PIN: u8 = 15;

    let mut freqmon = FrequencyMonitor::new(1);

    let mut selected_menu = Menu::Tests;
    let mut pot_values = [0; 3];
    let mut encoder_switch_states = [false; 3];

    loop {
        // Read inputs (buttons, pots, encoders)
        if !io_expander.digital_read(BUTTON1_PIN).unwrap() {
            selected_menu = Menu::Tests
        }
        if !io_expander.digital_read(BUTTON2_PIN).unwrap() {
            selected_menu = Menu::FrequencyMonitor;
            TRACKERS[2].store(100, Ordering::Relaxed);
        }

        let mut pot_leds = 0;
        let mut adc_text: String<72> = String::new();

        for (i, reader) in pot_readers.iter_mut().enumerate() {
            let read: u16 = adc.read(reader).unwrap();
            pot_values[i] = read;
        }

        if !io_expander.digital_read(BUTTON3_PIN).unwrap() {
            write!(adc_text, "{}", 3).unwrap();
        }
        if !io_expander.digital_read(BUTTON4_PIN).unwrap() {
            write!(adc_text, "{}", 4).unwrap();
        }

        for (enc_id, switch) in enc_switches.iter().enumerate() {
            encoder_switch_states[enc_id] = switch.is_low().unwrap();
        }

        // Render things on the screen / leds
        display.clear();

        match selected_menu {
            Menu::Tests => {
                for (i, read) in pot_values.iter().enumerate() {
                    let leds = ((read >> 10) & 0b11) as u8;
                    pot_leds |= leds << (i * 2);
                    write!(adc_text, "ADC{}: {:2} ", i, read / 64).unwrap();
                    write!(
                        adc_text,
                        "ENC{}: {}\n",
                        i,
                        TRACKERS[i].load(Ordering::Relaxed) as i32
                    )
                    .unwrap();
                }

                activity = (activity + 1) % activities.len();
                write!(adc_text, "{}\n", activities[activity]).unwrap();

                Text::with_baseline(&adc_text, Point::zero(), text_style, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                io_expander
                    .write_gpio(mcp23017::Port::GPIOA, pot_leds)
                    .unwrap();

                let mut enc_leds: u8 = 0;
                for (enc_id, switch) in enc_switches.iter().enumerate() {
                    if switch.is_low().unwrap() {
                        enc_leds |= 0b11 << (enc_id * 2);
                    }
                }

                io_expander
                    .write_gpio(mcp23017::Port::GPIOB, enc_leds)
                    .unwrap();
            }
            Menu::FrequencyMonitor => {
                let mut data = [0; FrequencyMonitor::FFT_SIZE];

                freqmon.inverse_scale =
                    TRACKERS[2].load(Ordering::Relaxed).clamp(1, 1000) * 2000000;

                critical_section::with(|cs| {
                    let t = FFT_MON_TIME_DOMAIN_DATA_MUTEX.borrow(cs).try_borrow();
                    if let Ok(t) = t {
                        data.copy_from_slice(t.as_ref());
                    }
                });

                freqmon.recompute(data);
                freqmon.draw(&mut display).unwrap();
            }
        }

        display.flush().unwrap();
    }
}

enum Menu {
    Tests,
    FrequencyMonitor,
}

type Encoder = RotaryEncoder<
    rotary_encoder_embedded::standard::StandardMode,
    Pin<DynPinId, FunctionSioInput, PullUp>,
    Pin<DynPinId, FunctionSioInput, PullUp>,
>;

static GLOBAL_ENCODERS: Mutex<RefCell<Option<[Encoder; 3]>>> = Mutex::new(RefCell::new(None));
static TRACKERS: [AtomicU32; 3] = [AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0)];
static ALARM: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));

#[interrupt]
#[allow(non_snake_case)]
fn TIMER_IRQ_0() {
    static mut ENCODER: Option<[Encoder; 3]> = None;

    if ENCODER.is_none() {
        cortex_m::interrupt::free(|cs| {
            *ENCODER = GLOBAL_ENCODERS.borrow(&cs).take();
        });
    }

    if let Some(encoders) = ENCODER {
        for (i, encoder) in encoders.iter_mut().enumerate() {
            match encoder.update() {
                Direction::None => (),
                Direction::Clockwise => {
                    let v = TRACKERS[i].load(Ordering::Acquire);
                    TRACKERS[i].store(v.overflowing_sub(1).0, Ordering::Release);
                }
                Direction::Anticlockwise => {
                    let v = TRACKERS[i].load(Ordering::Acquire);
                    TRACKERS[i].store(v.overflowing_add(1).0, Ordering::Release);
                }
            }
        }
    }

    cortex_m::interrupt::free(|cs| {
        let mut alarm = ALARM.borrow(cs).take().unwrap();
        alarm.clear_interrupt();
        alarm
            .schedule(Duration::<u32, 1, 1000000>::millis(2))
            .unwrap();
        alarm.enable_interrupt();
        ALARM.borrow(cs).replace(core::prelude::v1::Some(alarm));
    });
}
