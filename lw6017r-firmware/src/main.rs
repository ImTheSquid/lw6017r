#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(sync_unsafe_cell)]
#![feature(atomic_bool_fetch_not)]

use core::{
    cell::{Cell, RefCell, SyncUnsafeCell, UnsafeCell},
    ptr::addr_of_mut,
    sync::atomic::{AtomicBool, AtomicU8, Ordering},
};

use critical_section::Mutex;
use debouncr::{
    debounce_stateful_12, debounce_stateful_2, debounce_stateful_4, Debouncer, DebouncerStateful,
    Edge, Repeat12, Repeat2, Repeat4, Repeat6,
};
use embassy_executor::{raw::wake_task, Spawner};
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

type ButtonDebouncer = DebouncerStateful<u16, Repeat12>;

const READ_MASK: u8 = 1 << 6;
const ADDRESS_MASK: u8 = 7;
const ADDRESS_SHIFT: u8 = 0;
const PAGE_MASK: u8 = 0x18;
const PAGE_SHIFT: u8 = 3;
const FIXED_MASK: u8 = 1 << 5;

const DIGIT_PARSE_MAP: [u8; 10] = [
    0b00111111, 0b00000110, 0b10101101, 0b10101110, 0b10010110, 0b10111010, 0b10111011, 0b00100110,
    0b10111111, 0b10111110,
];

const PARSE_MASK: u8 = !(1 << 6);

const F_PARSE: u8 = 0b10110001;

const REMIND_MASK: u8 = 1 << 0;
const ENERGY_SAVER_FLAG: u8 = 1 << 2;
const DRY_FLAG: u8 = 1 << 3;
const TIMER_MASK: u8 = 1 << 5;
const FAN_FLAG: u8 = 1 << 6;
const COOL_FLAG: u8 = 1 << 7;
const MODE_MASK: u8 = ENERGY_SAVER_FLAG | DRY_FLAG | FAN_FLAG | COOL_FLAG;

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
    mode: Mode,
}

struct State {
    power_button: ButtonDebouncer,
    fan_button: ButtonDebouncer,
    mode_button: ButtonDebouncer,
    up_button: ButtonDebouncer,
    down_button: ButtonDebouncer,
    timer_button: ButtonDebouncer,
    showing_fan_speed: bool,
    on: bool,
    config: Config,
}

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

const DELAY: u64 = 100;

fn swap_endian(i: u8) -> u8 {
    let mut val = 0;

    if i & 0x01 != 0 {
        val |= 0x80
    }
    if i & 0x02 != 0 {
        val |= 0x40
    }
    if i & 0x04 != 0 {
        val |= 0x20
    }
    if i & 0x08 != 0 {
        val |= 0x10
    }
    if i & 0x10 != 0 {
        val |= 0x08
    }
    if i & 0x20 != 0 {
        val |= 0x04
    }
    if i & 0x40 != 0 {
        val |= 0x02
    }
    if i & 0x80 != 0 {
        val |= 0x01
    }

    val
}

static mut APP_CORE_STACK: Stack<4096> = Stack::new();

#[derive(Debug)]
enum ReceivedGlyph {
    Fan,
    Number(u8),
}

enum Error {
    BadParse,
}

fn parse_received(byte: u8) -> Result<ReceivedGlyph, Error> {
    for (i, itm) in DIGIT_PARSE_MAP.iter().enumerate() {
        if byte & PARSE_MASK == *itm {
            return Ok(ReceivedGlyph::Number(i as u8));
        }
    }
    match byte & PARSE_MASK {
        F_PARSE => Ok(ReceivedGlyph::Fan),
        _ => Err(Error::BadParse),
    }
}

// This item works as both the button register (24 bits) and the display reigster (32 bits)
// This allows for only one operation to get all data necessary instead of two
static LATEST_DATA: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);
static LATEST_BUTTON: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);
static LOCK: AtomicBool = AtomicBool::new(false);

#[link_section = ".iram1"]
fn main_comms(
    stb: Input<'static, Gpio5>,
    clk: Input<'static, Gpio4>,
    mut data: Flex<'static, Gpio2>,
) -> ! {
    let mut buttons = 0_u32;
    'forever: loop {
        let mut counter = 0_u32;
        let mut offset = 0_u8;
        // while stb.is_high() {}

        'stb: while stb.is_low() {
            println!("clk low");
            // Wait for clk to go low
            while clk.is_high() {
                if stb.is_high() {
                    break 'stb;
                }
            }

            counter |= (data.is_high() as u32) << offset;
            offset += 1;

            let counter_low = counter as u8;
            if offset == 8 && counter_low & READ_MASK != 0 {
                let fixed = counter_low & FIXED_MASK != 0;
                let address = (counter_low & ADDRESS_MASK) >> ADDRESS_SHIFT;
                let mut bit_offset = address * 8;

                data.set_as_output();

                // Wait for clock pin to go high, finishing the command write
                while clk.is_low() {}

                while stb.is_low() {
                    for i in bit_offset..bit_offset + 8 {
                        while clk.is_high() {}

                        data.set_level(if buttons >> i & 1 == 1 {
                            Level::High
                        } else {
                            Level::Low
                        });

                        // Data is clocked out at falling edge
                        while clk.is_low() {}
                    }

                    if !fixed {
                        bit_offset += 8;
                    }
                }

                data.set_as_output();
                continue 'forever;
            }

            if clk.is_high() {
                // We took too long, so just ignore this instruction
                println!("ctl");
                while stb.is_high() {}
                continue 'forever;
            }

            // Wait for clk to go high
            while clk.is_low() && stb.is_low() {}
        }

        // Store to shared memory
        if counter != 0 {
            if let Ok(false) =
                LOCK.compare_exchange(false, true, Ordering::SeqCst, Ordering::Acquire)
            {
                unsafe {
                    *LATEST_DATA.get() = counter;
                    counter = 0;

                    buttons = *LATEST_BUTTON.get();
                }
                LOCK.fetch_not(Ordering::AcqRel);
            } else {
                println!("LF");
            }
        }
    }
}

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

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Setup data lines
    let stb = Input::new(io.pins.gpio5, Pull::Up);
    let data = Flex::new(io.pins.gpio2);

    let clk = Input::new(io.pins.gpio4, Pull::Up);
    let mut irq_n = Output::new(io.pins.gpio6, Level::High);

    // Setup buttons
    let mut buttons = Flex::new(io.pins.gpio45);

    // Setup segments
    let mut digit1 = Output::new(io.pins.gpio7, Level::Low);
    let mut digit2 = Output::new(io.pins.gpio8, Level::Low);
    let mut leds = Output::new(io.pins.gpio9, Level::Low);
    let mut segments = Segments {
        seg0: AnyFlex::new(io.pins.gpio10),
        seg1: AnyFlex::new(io.pins.gpio11),
        seg2: AnyFlex::new(io.pins.gpio12),
        seg3: AnyFlex::new(io.pins.gpio13),
        seg4: AnyFlex::new(io.pins.gpio14),
        seg5: AnyFlex::new(io.pins.gpio17),
        seg6: AnyFlex::new(io.pins.gpio18),
        seg7: AnyFlex::new(io.pins.gpio21),
    };

    let mut state = State {
        up_button: debounce_stateful_12(false),
        down_button: debounce_stateful_12(false),
        power_button: debounce_stateful_12(false),
        mode_button: debounce_stateful_12(false),
        timer_button: debounce_stateful_12(false),
        fan_button: debounce_stateful_12(false),
        showing_fan_speed: false,
        on: true,
        config: Config {
            temperature: 00,
            fan_speed: FanSpeed::Low,
            mode: Mode::Dry,
        },
    };

    // critical_section::with(|cs| {
    //     stb.listen(Event::FallingEdge);
    //     STB.borrow_ref_mut(cs).replace(stb);
    //     DATA.borrow_ref_mut(cs).replace(data);
    //     CLK.borrow_ref_mut(cs).replace(clk);
    //     IRQ_N.borrow_ref_mut(cs).replace(irq_n);
    //     DISPLAY_CHANNEL_SEND
    //         .borrow_ref_mut(cs)
    //         .replace(display_channel_send);
    //     STATE.borrow_ref_mut(cs).replace(state);
    // });

    // Spin off display process
    let _guard = cpu.start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, move || {
        // static EXECUTOR: StaticCell<Executor> = StaticCell::new();
        // let executor = EXECUTOR.init(Executor::new());
        // executor.run(|spawner| {
        //     spawner
        //         .spawn(display_task(
        //             segments,
        //             digit1,
        //             digit2,
        //             leds,
        //             buttons,
        //             display_channel_recv,
        //             button_channel_send,
        //             buffer_channel_recv,
        //         ))
        //         .ok();
        // });

        log::info!("Starting comm thread");
        let _comm_enable = Output::new(io.pins.gpio1, Level::High);
        main_comms(stb, clk, data);
    });

    loop {
        segments.exec_on_all(|p| p.set_as_open_drain(Pull::None));
        let num = if state.showing_fan_speed {
            Some(match state.config.fan_speed {
                FanSpeed::High => 3,
                FanSpeed::Medium => 2,
                FanSpeed::Low => 1,
            })
        } else if state.on {
            Some(state.config.temperature)
        } else {
            None
        };
        if let Some(num) = num {
            digit1.set_high();
            if num <= 3 && num > 0 {
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
        segments.display_leds(
            if state.on {
                Some(state.config.mode)
            } else {
                None
            },
            false,
            true,
        );
        Timer::after_micros(DELAY).await;
        leds.set_low();

        segments.exec_on_all(|p| p.set_as_input(Pull::Down));
        buttons.set_as_output();
        buttons.set_high();
        let ButtonStatus {
            temp_up,
            temp_down,
            power,
            timer,
            mode,
            fan,
        } = segments.read_buttons();
        let interrupt = [
            state.power_button.update(power),
            state.fan_button.update(fan),
            state.up_button.update(temp_up),
            state.down_button.update(temp_down),
            state.mode_button.update(mode),
            state.timer_button.update(timer),
        ];

        let interrupt = interrupt
            .iter()
            .any(|e| e.is_some_and(|e| matches!(e, Edge::Rising)));

        if interrupt {
            irq_n.set_low();
        }
        Timer::after_micros(DELAY).await;
        buttons.set_as_open_drain(Pull::None);

        while LOCK
            .compare_exchange(false, true, Ordering::SeqCst, Ordering::Acquire)
            .is_err()
        {
            Timer::after_nanos(1).await;
        }

        let current_data = unsafe {
            let ptr = LATEST_DATA.get();
            let ret = *ptr;
            *ptr = 0;
            ret
        };

        LOCK.fetch_not(Ordering::SeqCst);

        if current_data != 0 {
            let buff = [
                current_data as u8,
                (current_data >> 8) as u8,
                (current_data >> 16) as u8,
                (current_data >> 24) as u8,
            ];

            println!("RECEIVED BUFFER {buff:?}");

            let page = (buff[0] & PAGE_MASK) >> PAGE_SHIFT;
            let address = (buff[0] & ADDRESS_MASK) >> ADDRESS_SHIFT;
            log::info!("{page}:{address}");
            match page {
                0 => {
                    println!("\n============================");
                    println!("RECEIVED DISPLAY BUFFER");
                    for itm in &buff {
                        println!("{itm:08b}");
                        if let Ok(ReceivedGlyph::Number(n)) = parse_received(*itm) {
                            println!("NUM: {n}")
                        }
                    }

                    // Display and LEDs
                    if buff.len() != 4 {
                        log::warn!("Display buffer is {} elements long, needed 4", buff.len());
                        continue;
                    }

                    // LEDs
                    let (mode, timer, remind) = if buff[1] != 0 {
                        let byte = buff[1];

                        let mode = match byte & MODE_MASK {
                            COOL_FLAG => Some(Mode::Cool),
                            ENERGY_SAVER_FLAG => Some(Mode::EnergySaver),
                            FAN_FLAG => Some(Mode::Fan),
                            DRY_FLAG => Some(Mode::Dry),
                            _ => {
                                log::warn!("Invalid mode 0b{:08b}", byte & MODE_MASK);
                                None
                            }
                        };

                        (mode, byte & TIMER_MASK != 0, byte & REMIND_MASK != 0)
                    } else {
                        (None, false, false)
                    };

                    if let Some(mode) = mode {
                        state.config.mode = mode;
                    }
                    // state.timer = if timer { Some(0) } else { None };
                    // state.remind = remind;

                    // If the display is off, skip this and turn off
                    if buff[2] == 0 && buff[3] == 0 {
                        state.on = false;
                        continue;
                    }

                    // First display element (right)
                    let lower = {
                        match parse_received(buff[2]) {
                            Ok(r) => match r {
                                ReceivedGlyph::Number(num) => Some(num),
                                ReceivedGlyph::Fan => {
                                    log::warn!("Received fan character for right element!");
                                    None
                                }
                            },
                            Err(_) => {
                                log::warn!("Invalid parse for right element!");
                                None
                            }
                        }
                    };

                    let lower = lower.unwrap_or(state.config.temperature % 10);

                    // Second display element (left)
                    let upper = {
                        match parse_received(buff[3]) {
                            Ok(r) => Some(r),
                            Err(_) => {
                                log::warn!("Invalid parse for left element!");
                                None
                            }
                        }
                    };

                    let upper =
                        upper.unwrap_or(ReceivedGlyph::Number(state.config.temperature / 10));

                    match upper {
                        ReceivedGlyph::Fan => {
                            state.config.fan_speed = match lower {
                                1 => FanSpeed::Low,
                                2 => FanSpeed::Medium,
                                3 => FanSpeed::High,
                                _ => {
                                    log::warn!("Invalid fan speed {lower}");
                                    continue;
                                }
                            };
                        }
                        ReceivedGlyph::Number(upper) => {
                            // state.config.fan_speed = None;
                            state.config.temperature = upper * 10 + lower;
                        }
                    }
                }
                1 => {
                    // Specialized display register for LEDs, unused for some reason
                    log::warn!("Unused page 1 address {address} specified in command!");
                }
                2 => {
                    log::info!("Display config {:?}", buff);
                }
                3 => log::warn!("Writing to LED configuration page, not used"),
                _ => log::warn!("Impossible page {page} address {address} specified!"),
            }
        }
    }

    // loop {
    //     // Check button status
    //     while stb.is_high() {
    //         if let Ok(ButtonStatus {
    //             temp_up,
    //             temp_down,
    //             power,
    //             timer,
    //             mode,
    //             fan,
    //         }) = button_channel_recv.try_receive()
    //         {

    //         }
    //     }

    //     // If the controller requests a READ, get some data ready for it in the form of a u32 (lower 24 used)
    //     let read_data: u32 = {
    //         let addr0 = 0;

    //         let addr1 = 0;

    //         let addr2 = 0;

    //         addr0 | (addr1 << 8) | (addr2 << 16)
    //     };

    //     data.set_as_input(Pull::Up);

    //     let mut baker = ByteBaker::default();

    //     let mut buffer: Vec<_, 16> = heapless::Vec::new();

    //     let mut counter = 0_u64;

    //     while stb.is_low() {
    //         // Wait for clk to go low
    //         while clk.is_high() && stb.is_low() {}

    //         // // If stb went high, break
    //         if stb.is_high() {
    //             break;
    //         }

    //         if counter < 100000 {
    //             counter += 1;
    //         } else {
    //             counter = 0;
    //         }

    //         // Read some data
    //         // if let Some(byte) = baker.bake(data.is_high()) {
    //         //     if byte & READ_MASK != 0 && buffer.is_empty() {
    //         //         let fixed = byte & FIXED_MASK != 0;
    //         //         let address = (byte & ADDRESS_MASK) >> ADDRESS_SHIFT;
    //         //         let mut bit_offset = address * 8;

    //         //         data.set_as_output();

    //         //         // Wait for clock pin to go high, finishing the command write
    //         //         while clk.is_low() && stb.is_low() {}

    //         //         if stb.is_high() {
    //         //             break;
    //         //         }

    //         //         while stb.is_low() {
    //         //             for i in 0_u8..8 {
    //         //                 while clk.is_high() && stb.is_low() {}

    //         //                 if stb.is_high() {
    //         //                     break;
    //         //                 }

    //         //                 data.set_level(if read_data >> (i + bit_offset) & 1 == 1 {
    //         //                     Level::High
    //         //                 } else {
    //         //                     Level::Low
    //         //                 });

    //         //                 // Data is clocked out at falling edge
    //         //                 while clk.is_low() && stb.is_low() {}

    //         //                 if stb.is_high() {
    //         //                     break;
    //         //                 }
    //         //             }

    //         //             if !fixed {
    //         //                 bit_offset += 8;
    //         //             }
    //         //         }
    //         //     } else {
    //         //         buffer.push(byte).expect("byte to have room to push");
    //         //     }
    //         // }

    //         if clk.is_high() {
    //             println!("TOO SLOW");
    //         }

    //         // Wait for clk to go high
    //         while clk.is_low() && stb.is_low() {}
    //     }

    //     // Try to send over the latest buffer, but if can't then just ignore
    //     if !buffer.is_empty() {
    //         log::info!("send");
    //         let _ = buffer_channel_send.try_send(buffer);
    //     }
    // }
}

// /// Bakes a byte by adding in bits
// #[derive(Debug, Default)]
// struct ByteBaker {
//     byte: u8,
//     offset: u8,
// }

// impl ByteBaker {
//     #[inline(always)]
//     fn bake(&mut self, bit: bool) -> Option<u8> {
//         self.byte |= (bit as u8) << self.offset;

//         if self.offset == 7 {
//             let copy = self.byte;
//             self.byte = 0;
//             self.offset = 0;
//             Some(copy)
//         } else {
//             self.offset += 1;
//             None
//         }
//     }
// }
