#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::{cell::RefCell, ptr::addr_of_mut};

use critical_section::Mutex;
use debouncr::{
    debounce_stateful_12, debounce_stateful_2, debounce_stateful_4, Debouncer, DebouncerStateful,
    Edge, Repeat12, Repeat2, Repeat4, Repeat6,
};
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
        Gpio21, Gpio4, Gpio45, Gpio5, Gpio6, Gpio7, Gpio8, Gpio9, Input, Io, Level, Output, Pull,
    },
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
};
use esp_hal_embassy::Executor;
use esp_println::println;
use heapless::Vec;
use static_cell::{make_static, StaticCell};

type ButtonDebouncer = DebouncerStateful<u8, Repeat2>;

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
    on: bool,
}

struct State {
    power_button: ButtonDebouncer,
    fan_button: ButtonDebouncer,
    mode_button: ButtonDebouncer,
    up_button: ButtonDebouncer,
    down_button: ButtonDebouncer,
    timer_button: ButtonDebouncer,
    config: Config,
}

static STB: Mutex<RefCell<Option<Input<Gpio5>>>> = Mutex::new(RefCell::new(None));
static DATA: Mutex<RefCell<Option<Flex<Gpio2>>>> = Mutex::new(RefCell::new(None));
static CLK: Mutex<RefCell<Option<Input<Gpio4>>>> = Mutex::new(RefCell::new(None));
static IRQ_N: Mutex<RefCell<Option<Output<Gpio6>>>> = Mutex::new(RefCell::new(None));
static STATE: Mutex<RefCell<Option<State>>> = Mutex::new(RefCell::new(None));
static DISPLAY_CHANNEL_SEND: Mutex<
    RefCell<Option<Sender<CriticalSectionRawMutex, DisplayState, 1>>>,
> = Mutex::new(RefCell::new(None));

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
        self.exec_on_some(|p| p.set_low(), power);
        self.exec_on_some(|p| p.set_high(), !power);
    }

    fn display_number(&mut self, num: u8, dp: bool) {
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

    fn display_letter(&mut self, c: char, dp: bool) {
        self.byte_power(
            match c {
                'F' => 0b01110001,
                _ => unimplemented!(),
            } | if dp { 1 << 7 } else { 0 },
        );
    }

    fn display_leds(&mut self, mode: Option<Mode>, timer: bool, remind: bool) {
        let byte = if timer { 1 << 5 } else { 0 }
            | if remind { 1 << 4 } else { 0 }
            | if let Some(mode) = mode {
                match mode {
                    Mode::Cool => 1,
                    Mode::EnergySaver => 1 << 1,
                    Mode::Fan => 1 << 2,
                    Mode::Dry => 1 << 3,
                }
            } else {
                0
            };

        self.byte_power(byte);
    }

    fn read_buttons(&mut self) -> ButtonStatus {
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
    mode: Option<Mode>,
    fan: Option<FanSpeed>,
}

const DELAY: u64 = 750;

#[embassy_executor::task]
async fn display_task(
    mut segments: Segments<'static>,
    mut digit1: Output<'static, Gpio7>,
    mut digit2: Output<'static, Gpio8>,
    mut leds: Output<'static, Gpio9>,
    mut buttons: Flex<'static, Gpio45>,
    state_recv: Receiver<'static, CriticalSectionRawMutex, DisplayState, 1>,
    button_alert: Sender<'static, CriticalSectionRawMutex, ButtonStatus, 1>,
) -> ! {
    let mut state = DisplayState::default();

    loop {
        segments.exec_on_all(|p| p.set_as_open_drain(Pull::None));
        let num = if let Some(fan) = state.fan {
            Some(match fan {
                FanSpeed::High => 3,
                FanSpeed::Medium => 2,
                FanSpeed::Low => 1,
            })
        } else if state.mode.is_some() {
            Some(state.temp)
        } else {
            state.timer
        };
        if let Some(num) = num {
            digit1.set_high();
            if num <= 3 {
                segments.display_letter('F', false);
            } else {
                segments.display_number(num / 10, false);
            }
            Timer::after_micros(DELAY).await;
            digit1.set_low();

            digit2.set_high();
            segments.display_number(num % 10, false);
            Timer::after_micros(DELAY).await;
            digit2.set_low();
        }

        leds.set_high();
        segments.display_leds(state.mode, state.timer.is_some(), state.remind);
        Timer::after_micros(DELAY).await;
        leds.set_low();

        segments.exec_on_all(|p| p.set_as_input(Pull::Down));
        buttons.set_as_output();
        buttons.set_high();
        let button = segments.read_buttons();
        Timer::after_micros(DELAY).await;
        buttons.set_as_open_drain(Pull::None);
        let _ = button_alert.try_send(button);

        if let Ok(s) = state_recv.try_receive() {
            state = s;
        }
    }
}

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
    let mut stb = Input::new(io.pins.gpio5, Pull::Up);
    let mut data = Flex::new(io.pins.gpio2);
    data.set_as_input(Pull::Up);
    let clk = Input::new(io.pins.gpio4, Pull::None);
    let irq_n = Output::new(io.pins.gpio6, Level::High);

    // Setup buttons
    let buttons = Flex::new(io.pins.gpio45);

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
    let display_channel_send = display_channel.sender();
    let display_channel_recv = display_channel.receiver();
    let button_channel: Channel<CriticalSectionRawMutex, ButtonStatus, 1> = Channel::new();
    let button_channel = make_static!(button_channel);
    let button_channel_send = button_channel.sender();
    let button_channel_recv = button_channel.receiver();

    let state = State {
        up_button: debounce_stateful_2(false),
        down_button: debounce_stateful_2(false),
        power_button: debounce_stateful_2(false),
        mode_button: debounce_stateful_2(false),
        timer_button: debounce_stateful_2(false),
        fan_button: debounce_stateful_2(false),
        config: Config {
            temperature: 72,
            fan_speed: FanSpeed::Low,
            timer: 0,
            replace: true,
            mode: Mode::EnergySaver,
            on: true,
        },
    };

    display_channel_send
        .send(DisplayState {
            temp: state.config.temperature,
            remind: state.config.replace,
            timer: None,
            mode: Some(state.config.mode),
            fan: None,
        })
        .await;

    critical_section::with(|cs| {
        stb.listen(Event::FallingEdge);
        STB.borrow_ref_mut(cs).replace(stb);
        DATA.borrow_ref_mut(cs).replace(data);
        CLK.borrow_ref_mut(cs).replace(clk);
        IRQ_N.borrow_ref_mut(cs).replace(irq_n);
        DISPLAY_CHANNEL_SEND
            .borrow_ref_mut(cs)
            .replace(display_channel_send);
        STATE.borrow_ref_mut(cs).replace(state);
    });

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

    let _comm_enable = Output::new(io.pins.gpio1, Level::High);

    loop {
        log::info!("HEARTBEAT");
        Timer::after_millis(500).await;
        if let Ok(ButtonStatus {
            temp_up,
            temp_down,
            power,
            timer,
            mode,
            fan,
        }) = button_channel_recv.try_receive()
        {
            println!("received button update {temp_up} {temp_down} {power} {timer} {mode} {fan}");
            critical_section::with(|cs| {
                let mut binding = STATE.borrow_ref_mut(cs);
                let state = binding.as_mut().unwrap();

                let interrupt = [
                    state.power_button.update(power),
                    state.fan_button.update(fan),
                    state.up_button.update(temp_up),
                    state.down_button.update(temp_down),
                    state.mode_button.update(mode),
                    state.timer_button.update(timer),
                ]
                .iter()
                .any(|e| e.is_some_and(|e| matches!(e, Edge::Rising)));

                if interrupt {
                    let mut binding = IRQ_N.borrow_ref_mut(cs);
                    let irq_n = binding.as_mut().unwrap();
                    irq_n.set_high();
                }
            });
        }
    }
}

#[allow(unused)]
#[derive(Debug)]
enum Order {
    MostSignificant,
    LeastSignificant,
}

/// Bakes a byte by adding in bits
#[derive(Debug)]
struct ByteBaker {
    byte: u8,
    offset: u8,
    order: Order,
}

impl ByteBaker {
    fn new(order: Order) -> Self {
        Self {
            byte: 0,
            offset: 0,
            order,
        }
    }

    fn bake(&mut self, bit: bool) -> Option<u8> {
        let order_offset = match self.order {
            Order::LeastSignificant => self.offset,
            Order::MostSignificant => 7 - self.offset,
        };

        self.byte |= (bit as u8) << order_offset;

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

#[derive(Debug)]
enum CommandParseState {
    NoCommand,
    Write { fixed: bool, address: u8, page: u8 },
    Read { fixed: bool, address: u8, page: u8 },
}

fn validate_write_address(address: u8, page: u8) {
    let res = match page {
        // 7 segment
        0b00 => address <= 0x05,
        // LED
        0b01 => address == 0,
        // Config
        0b10 | 0b11 => address <= 0x03,
        _ => false,
    };

    if !res {
        panic!("Invalid write to {address} on page {page}");
    }
}

fn validate_read_address(address: u8, page: u8) {
    let res = page == 1 && address <= 2;

    if !res {
        panic!("Invalid read from {address} on page {page}");
    }
}

#[handler(priority = esp_hal::interrupt::Priority::Priority3)]
fn stb_handler() {
    // println!("interrupt");
    critical_section::with(|cs| {
        // Start handling clock bits
        let mut binding = STB.borrow_ref_mut(cs);
        let stb = binding.as_mut().unwrap();
        let mut binding = DATA.borrow_ref_mut(cs);
        let data = binding.as_mut().unwrap();
        let binding = CLK.borrow_ref(cs);
        let clk = binding.as_ref().unwrap();
        let mut binding = STATE.borrow_ref_mut(cs);
        let state = binding.as_mut().unwrap();
        let mut binding = IRQ_N.borrow_ref_mut(cs);
        let irq_n = binding.as_mut().unwrap();

        data.set_as_input(Pull::Up);

        let mut baker = ByteBaker::new(Order::LeastSignificant);

        let mut parse_state = CommandParseState::NoCommand;

        while stb.is_low() {
            // Wait for clk to go low
            while clk.is_high() && stb.is_low() {}

            // If stb went high, break
            if stb.is_high() {
                break;
            }

            // Read some data
            if let Some(byte) = baker.bake(data.is_high()) {
                // println!("parsed byte 0b{byte:08b}");
                // Check for obvious commands first, then fallback to state
                match parse_state {
                    CommandParseState::NoCommand => match byte & 0b01011111 {
                        0b00001101 => {
                            state.config.on = true;
                        }
                        0b00001110 => {
                            state.config.on = false;
                        }
                        _ => {
                            // At this point it has to be a read or write command, so extract necessary info
                            let fixed = byte & 0b00100000 > 0;
                            let address = byte & 0b111;
                            let page = (byte & 0b11000) >> 3;

                            if byte & 0b01000000 > 0 {
                                parse_state = CommandParseState::Read {
                                    fixed,
                                    address,
                                    page,
                                };
                            } else {
                                parse_state = CommandParseState::Write {
                                    fixed,
                                    address,
                                    page,
                                };
                            }

                            // println!("parsed command {parse_state:?}");
                        }
                    },
                    CommandParseState::Read {
                        fixed,
                        address,
                        page,
                    } => {
                        validate_read_address(address, page);

                        data.set_as_output();

                        let out = match address {
                            0 => {
                                state.config.replace as u8
                                    | (matches!(state.config.mode, Mode::EnergySaver) as u8) << 2
                                    | (matches!(state.config.mode, Mode::Fan) as u8) << 3
                                    | ((state.config.timer > 0) as u8) << 5
                                    | (matches!(state.config.mode, Mode::Dry) as u8) << 6
                                    | (matches!(state.config.mode, Mode::Cool) as u8) << 7
                            }
                            1 => {
                                state.fan_button.is_high() as u8
                                    | (state.down_button.is_high() as u8) << 2
                                    | (state.up_button.is_high() as u8) << 3
                            }
                            2 => {
                                state.timer_button.is_high() as u8
                                    | (state.mode_button.is_high() as u8) << 1
                                    | (state.power_button.is_high() as u8) << 2
                            }
                            _ => unreachable!(),
                        };

                        for i in 0_u8..8 {
                            while clk.is_low() && stb.is_low() {}

                            if stb.is_high() {
                                break;
                            }

                            data.set_level(if out >> i & 1 == 1 {
                                Level::High
                            } else {
                                Level::Low
                            });

                            // Data is clocked out at falling edge
                            while clk.is_high() && stb.is_low() {}

                            if stb.is_high() {
                                break;
                            }
                        }

                        data.set_as_input(Pull::Up);

                        if address == 2 {
                            irq_n.set_low();
                        }

                        if !fixed {
                            parse_state = CommandParseState::Read {
                                fixed,
                                address: address + 1,
                                page,
                            };
                        }
                    }
                    CommandParseState::Write {
                        fixed,
                        address,
                        page,
                    } => {
                        validate_write_address(address, page);

                        match page {
                            0 => {
                                if address <= 1 {
                                    println!("recv display byte 0b{byte:08b}");
                                } else {
                                    println!("Received display byte 0b{byte:08b} at address 0x{address:02x} outside of displayable range, ignoring");
                                }
                            }
                            1 => {
                                if byte & 0b11001100 != 0 {
                                    state.config.mode = match byte & 0b11001100 {
                                        128 => Mode::Cool,
                                        64 => Mode::Dry,
                                        8 => Mode::Fan,
                                        4 => Mode::EnergySaver,
                                        _ => unreachable!(),
                                    };
                                    state.config.on = true;
                                } else {
                                    state.config.on = false;
                                }

                                state.config.replace = byte & 1 != 0;

                                // Deal with timer elsewhere
                            }
                            0x10 | 0x11 => {
                                // Config information, not necessary
                                println!(
                                    "Received config byte 0b{byte:08b} at page 0x{page:02x} address 0x{address:02x}, ignoring"
                                );
                            }
                            _ => unreachable!(),
                        }

                        if !fixed {
                            parse_state = CommandParseState::Write {
                                fixed,
                                address: address + 1,
                                page,
                            };
                        }
                    }
                }
            }

            // Wait for clk to go high
            while clk.is_low() && stb.is_low() {}
        }

        stb.clear_interrupt();
    });
}
