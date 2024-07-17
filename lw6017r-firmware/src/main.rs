#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::{cell::RefCell, ptr::addr_of_mut};

use critical_section::Mutex;
use debouncr::{Debouncer, DebouncerStateful, Repeat12};
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, Receiver, Sender},
};
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal_async::delay::DelayNs;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    cpu_control::{CpuControl, Stack},
    delay::Delay,
    gpio::{
        AnyFlex, Event, Flex, Gpio10, Gpio11, Gpio12, Gpio13, Gpio14, Gpio17, Gpio18, Gpio2,
        Gpio21, Gpio4, Gpio45, Gpio5, Gpio7, Gpio8, Gpio9, Input, Io, Level, Output, Pull,
    },
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
};
use esp_hal_embassy::Executor;
use heapless::Vec;
use static_cell::{make_static, StaticCell};

type ButtonDebouncer = DebouncerStateful<u16, Repeat12>;

#[derive(Debug, Clone, Copy)]
enum FanSpeed {
    Low,
    Medium,
    High,
}

#[derive(Debug, Clone, Copy, Default)]
enum Mode {
    Cool,
    #[default]
    EnergySaver,
    Fan,
    Dry,
}

struct Config {
    temperature: u8,
    fan_speed: FanSpeed,
    timer: u8,
    replace: bool,
    mode: Mode,
}

struct State {
    power_button: ButtonDebouncer,
    fan_button: ButtonDebouncer,
    mode_button: ButtonDebouncer,
}

static STB: Mutex<RefCell<Option<Input<Gpio5>>>> = Mutex::new(RefCell::new(None));
static DATA: Mutex<RefCell<Option<Input<Gpio2>>>> = Mutex::new(RefCell::new(None));
static CLK: Mutex<RefCell<Option<Input<Gpio4>>>> = Mutex::new(RefCell::new(None));

struct Segments<'d> {
    seg0: AnyFlex<'d>,
    seg1: AnyFlex<'d>,
    seg2: AnyFlex<'d>,
    seg3: AnyFlex<'d>,
    seg4: AnyFlex<'d>,
    seg5: AnyFlex<'d>,
    seg6: AnyFlex<'d>,
    seg7: AnyFlex<'d>,
}

struct ButtonStatus {
    temp_up: bool,
    temp_down: bool,
    power: bool,
    timer: bool,
    mode: bool,
    fan: bool,
}

impl Segments<'_> {
    fn exec_on_some<F>(&mut self, mut f: F, flag: u8)
    where
        F: FnMut(&mut AnyFlex),
    {
        if flag & 1 == 1 {
            f(&mut self.seg0);
        }
        if flag >> 1 & 1 == 1 {
            f(&mut self.seg1);
        }
        if flag >> 2 & 1 == 1 {
            f(&mut self.seg2);
        }
        if flag >> 3 & 1 == 1 {
            f(&mut self.seg3);
        }
        if flag >> 4 & 1 == 1 {
            f(&mut self.seg4);
        }
        if flag >> 5 & 1 == 1 {
            f(&mut self.seg5);
        }
        if flag >> 6 & 1 == 1 {
            f(&mut self.seg6);
        }
        if flag >> 7 & 1 == 1 {
            f(&mut self.seg7);
        }
    }

    fn exec_on_all<F>(&mut self, f: F)
    where
        F: FnMut(&mut AnyFlex),
    {
        self.exec_on_some(f, 0xFF);
    }

    fn byte_power(&mut self, power: u8) {
        self.exec_on_some(|p| p.set_high(), power);
        self.exec_on_some(|p| p.set_low(), !power);
    }

    fn display_number(&mut self, num: u8, dp: bool) {
        self.exec_on_all(|p| p.set_as_output());

        self.byte_power(
            match num {
                0 => 0b00111111,
                1 => 0b00000110,
                2 => 0b01011011,
                3 => 0b01001111,
                4 => 0b01100110,
                5 => 0b01101101,
                6 => 0b01111101,
                7 => 0b00000111,
                8 => 0b01111111,
                9 => 0b01101111,
                _ => unreachable!(),
            } | if dp { 0b10000000 } else { 0 },
        );
    }

    fn display_leds(&mut self, mode: Mode, timer: bool, remind: bool) {
        self.exec_on_all(|p| p.set_as_output());

        let byte = 0_u8
            | if timer { 1 << 5 } else { 0 }
            | if remind { 1 << 4 } else { 0 }
            | match mode {
                Mode::Cool => 1,
                Mode::EnergySaver => 1 << 1,
                Mode::Fan => 1 << 2,
                Mode::Dry => 1 << 3,
            };

        self.byte_power(byte);
    }

    fn read_buttons(&mut self) -> ButtonStatus {
        self.exec_on_all(|p| p.set_as_input(Pull::Down));

        ButtonStatus {
            temp_up: self.seg4.is_high(),
            temp_down: self.seg5.is_high(),
            power: self.seg0.is_high(),
            timer: self.seg2.is_high(),
            mode: self.seg1.is_high(),
            fan: self.seg3.is_high(),
        }
    }
}

#[derive(Debug, Default)]
struct DisplayState {
    temp: u8,
    remind: bool,
    timer: Option<u8>,
    is_on: bool,
    mode: Mode,
}

#[embassy_executor::task]
async fn display_task(
    mut segments: Segments<'static>,
    mut digit1: Output<'static, Gpio7>,
    mut digit2: Output<'static, Gpio8>,
    mut leds: Output<'static, Gpio9>,
    mut buttons: Output<'static, Gpio45>,
    state_recv: Receiver<'static, CriticalSectionRawMutex, DisplayState, 1>,
    button_alert: Sender<'static, CriticalSectionRawMutex, ButtonStatus, 1>,
) -> ! {
    let mut state = DisplayState::default();

    loop {
        let num = if state.is_on {
            Some(state.temp)
        } else if let Some(time) = state.timer {
            Some(time)
        } else {
            None
        };
        if let Some(num) = num {
            digit1.set_high();
            segments.display_number(num % 10, false);
            Timer::after_micros(64).await;
            digit1.set_low();

            digit2.set_high();
            segments.display_number(num / 10, false);
            Timer::after_micros(64).await;
            digit2.set_low();
        }

        leds.set_high();
        segments.display_leds(state.mode, state.timer.is_some(), state.remind);
        Timer::after_micros(64).await;
        leds.set_low();

        buttons.set_high();
        let button = segments.read_buttons();
        Timer::after_micros(64).await;
        buttons.set_low();
        button_alert.send(button).await;

        if let Ok(s) = state_recv.try_receive() {
            state = s;
        }
    }
}

fn send_data() {}

static mut APP_CORE_STACK: Stack<4096> = Stack::new();

#[main]
async fn main(_spawner: Spawner) -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let timer0 = OneShotTimer::new(timg0.timer0.into());
    let timer1 = OneShotTimer::new(timg0.timer1.into());
    let timers = [timer0, timer1];
    let timers = make_static!(timers);
    esp_hal_embassy::init(&clocks, timers);

    esp_println::logger::init_logger_from_env();

    let mut cpu = CpuControl::new(peripherals.CPU_CTRL);

    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    io.set_interrupt_handler(stb_handler);

    // Setup data lines
    let _comm_enable = Output::new(io.pins.gpio1, Level::High);
    let mut stb = Input::new(io.pins.gpio5, Pull::Up);
    let data = Input::new(io.pins.gpio2, Pull::Up);
    let clk = Input::new(io.pins.gpio4, Pull::None);
    let irq_n = Output::new(io.pins.gpio6, Level::High);

    // STB interrupt to get data from controller
    critical_section::with(|cs| {
        stb.listen(Event::FallingEdge);
        STB.borrow_ref_mut(cs).replace(stb);
        DATA.borrow_ref_mut(cs).replace(data);
        CLK.borrow_ref_mut(cs).replace(clk);
    });

    // Setup buttons
    let buttons = Output::new(io.pins.gpio45, Level::Low);

    // Setup segments
    let digit1 = Output::new(io.pins.gpio7, Level::Low);
    let digit2 = Output::new(io.pins.gpio8, Level::Low);
    let leds = Output::new(io.pins.gpio9, Level::Low);
    let segments = Segments {
        seg0: AnyFlex::new(io.pins.gpio10),
        seg1: AnyFlex::new(io.pins.gpio11),
        seg2: AnyFlex::new(io.pins.gpio12),
        seg3: AnyFlex::new(io.pins.gpio13),
        seg4: AnyFlex::new(io.pins.gpio14),
        seg5: AnyFlex::new(io.pins.gpio17),
        seg6: AnyFlex::new(io.pins.gpio18),
        seg7: AnyFlex::new(io.pins.gpio21),
    };

    // Channels
    let display_channel: Channel<CriticalSectionRawMutex, DisplayState, 1> = Channel::new();
    let display_channel = make_static!(display_channel);
    let display_channel_recv = display_channel.receiver();
    let button_channel: Channel<CriticalSectionRawMutex, ButtonStatus, 1> = Channel::new();
    let button_channel = make_static!(button_channel);
    let button_channel_send = button_channel.sender();

    // Spin off display process
    let _guard = cpu.start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, move || {
        static EXECUTOR: StaticCell<Executor> = StaticCell::new();
        let executor = EXECUTOR.init(Executor::new());
        executor.run(|spawner| {
            spawner
                .spawn(display_task(
                    segments,
                    digit1,
                    digit2,
                    leds,
                    buttons,
                    display_channel_recv,
                    button_channel_send,
                ))
                .ok();
        });
    });

    loop {
        log::info!("Hello world!");
        Timer::after_millis(500).await;
    }
}

/// Bakes a byte by adding in bits
#[derive(Debug, Default)]
struct ByteBaker {
    byte: u8,
    offset: u8,
}

const MAX_COMMAND_LEN: usize = 9;

impl ByteBaker {
    fn bake(&mut self, bit: bool) -> Option<u8> {
        self.byte |= (bit as u8) << self.offset;

        if self.offset == 7 {
            let copy = self.byte;
            self.byte = 0;
            self.offset = 0;
            Some(copy)
        } else {
            self.offset += 1;
            None
        }
    }
}

#[handler]
fn stb_handler() {
    critical_section::with(|cs| {
        // Start handling clock bits
        let mut binding = STB.borrow_ref_mut(cs);
        let stb = binding.as_mut().unwrap();
        let binding = DATA.borrow_ref(cs);
        let data = binding.as_ref().unwrap();
        let binding = CLK.borrow_ref(cs);
        let clk = binding.as_ref().unwrap();

        let mut recv_buf: Vec<u8, MAX_COMMAND_LEN> = Vec::new();
        let mut baker = ByteBaker::default();

        while stb.is_low() {
            // Wait for clk to go low
            while clk.is_high() && stb.is_low() {}

            // If stb went high, break
            if stb.is_high() {
                break;
            }

            // Read some data
            if let Some(byte) = baker.bake(data.is_high()) {
                recv_buf.push(byte).expect("byte should have room to push");
            }

            // Wait for clk to go high
            while clk.is_low() && stb.is_low() {}
        }

        stb.clear_interrupt();
    });
}
