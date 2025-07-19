#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use common::consts::*;
use core::cell::RefCell;
use core::fmt::Write as _;
use core::sync::atomic::AtomicI32;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;
use core::u32;
use cortex_m::interrupt::Mutex;
use cortex_m::singleton;
use defmt::info;
use defmt_rtt as _;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Baseline;
use embedded_graphics::text::Text;
use embedded_hal::adc::OneShot;
use embedded_hal::blocking::i2c::{Write, WriteRead};
use embedded_hal::digital::v2::InputPin;
use fixed::types::{I1F15, U8F8};
use fugit::Duration;
use fugit::HertzU32;
use fugit::RateExtU32;
use heapless::String;
use mcp23017::MCP23017;
use panic_probe as _;
use rotary_encoder_embedded::Direction;
use rotary_encoder_embedded::RotaryEncoder;
use rp2040_hal::gpio::FunctionSioInput;
use rp2040_hal::gpio::PullUp;
use rp2040_hal::pac::NVIC;
use rp2040_hal::timer::Alarm;
use rp2040_hal::timer::Alarm0;
use rp2040_hal::Timer;
use rp2040_hal::{
    adc::AdcPin,
    clocks::{Clock, ClockSource, ClocksManager, InitError},
    dma::{double_buffer, DMAExt},
    gpio::{self, DynFunction, DynPinId, Pin, PullDown},
    multicore::{Multicore, Stack},
    pac::interrupt,
    pac::{self},
    pio::{PIOExt, PinDir},
    pll::{common_configs::PLL_USB_48MHZ, setup_pll_blocking},
    sio::Sio,
    watchdog::Watchdog,
    xosc::setup_xosc_blocking,
    Adc, I2C,
};

use rytmos_synth::effect::{
    amplify::{Amplify, AmplifySettings},
    Effect,
};
use sh1106::mode::GraphicsMode;

static mut CORE1_STACK: Stack<4096> = Stack::new();

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

    let mut effect = Amplify::make(
        0,
        AmplifySettings {
            amplification: U8F8::from_num(U8F8::MAX / 8),
            // amplification: U8F8::from_num(1),
        },
    );
    let mut sample: I1F15;

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

    let pins = gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Setup the other core
    let sys_freq = clocks.system_clock.freq();
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    #[allow(static_mut_refs)]
    let _ = core1.spawn(unsafe { CORE1_STACK.take().unwrap() }, move || {
        setup_dual_adc_and_dac(sys_freq)
        // setup_adc_only(clocks.system_clock.freq())
    });

    info!("Set up at sys_freq = {}Hz", sys_freq.to_Hz());
    info!("Start I/O thread.");

    // set up the encoders on a timer, maybe using some atomic int?
    // set up the potentiometers (look at drum machine)
    // set up IO expander

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

    // let pullup_state = io_expander
    //     .read_register(mcp23017::Register::GPPUA)
    //     .unwrap();

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

    loop {
        display.clear();
        // let banks = io_expander.read_gpioab().unwrap();
        // info!("banks = {:016b}", banks);
        let mut pot_leds = 0;

        let mut adc_text: String<72> = String::new();

        for (i, reader) in pot_readers.iter_mut().enumerate() {
            let read: u16 = adc.read(reader).unwrap();
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

        if !io_expander.digital_read(BUTTON1_PIN).unwrap() {
            write!(adc_text, "{}", 1).unwrap();
        }
        if !io_expander.digital_read(BUTTON2_PIN).unwrap() {
            write!(adc_text, "{}", 2).unwrap();
        }
        if !io_expander.digital_read(BUTTON3_PIN).unwrap() {
            write!(adc_text, "{}", 3).unwrap();
        }
        if !io_expander.digital_read(BUTTON4_PIN).unwrap() {
            write!(adc_text, "{}", 4).unwrap();
        }

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
                // info!("enc switch {} triggeder", enc_id)
            }
        }

        io_expander
            .write_gpio(mcp23017::Port::GPIOB, enc_leds)
            .unwrap();

        display.flush().unwrap();
    }
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
