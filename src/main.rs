#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_halt as _;

use core::cell::RefCell;
use cortex_m::{interrupt::Mutex, prelude::_embedded_hal_adc_OneShot};
use cortex_m_rt::entry;

use microbit::{
    adc::{Adc, AdcConfig, Default},
    board::Board,
    display::nonblocking::{BitImage, Display},
    gpio::EDGE00,
    hal::{
        gpio::{Floating, Input, Level},
        gpiote::*,
        pac::{self, interrupt, TIMER0, TIMER1},
        ppi::{self, ConfigurablePpi, Ppi},
        prelude::*,
    },
};
struct Analog {
    converter: Adc,
    pin: EDGE00<Input<Floating>>,
}

const SMILE: BitImage = BitImage::new(&[
    [0, 1, 0, 1, 0],
    [1, 0, 1, 0, 1],
    [0, 0, 1, 0, 0],
    [1, 0, 0, 0, 1],
    [0, 1, 1, 1, 0],
]);

const ARROW_LEFT: BitImage = BitImage::new(&[
    [0, 0, 1, 0, 0],
    [0, 1, 0, 0, 0],
    [1, 1, 1, 1, 1],
    [0, 1, 0, 0, 0],
    [0, 0, 1, 0, 0],
]);

const ARROW_RIGHT: BitImage = BitImage::new(&[
    [0, 0, 1, 0, 0],
    [0, 0, 0, 1, 0],
    [1, 1, 1, 1, 1],
    [0, 0, 0, 1, 0],
    [0, 0, 1, 0, 0],
]);

const ARROW_DOWN: BitImage = BitImage::new(&[
    [0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0],
    [1, 0, 1, 0, 1],
    [0, 1, 1, 1, 0],
    [0, 0, 1, 0, 0],
]);

const ARROW_UP: BitImage = BitImage::new(&[
    [0, 0, 1, 0, 0],
    [0, 1, 1, 1, 0],
    [1, 0, 1, 0, 1],
    [0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0],
]);

enum CarState {
    Stopped,
    Forward,
    Left,
    Right,
    Back,
}

struct StateSpeed {
    state: CarState,
    lspeed: u32,
    rspeed: u32,
}

const STATE_STOPPED: StateSpeed = StateSpeed {
    state: CarState::Stopped,
    lspeed: 1500,
    rspeed: 1500,
};
const STATE_FORWARD: StateSpeed = StateSpeed {
    state: CarState::Forward,
    lspeed: 2500,
    rspeed: 500,
};
const STATE_BACK: StateSpeed = StateSpeed {
    state: CarState::Back,
    lspeed: 500,
    rspeed: 2500,
};
const STATE_LEFT: StateSpeed = StateSpeed {
    state: CarState::Left,
    lspeed: 2500,
    rspeed: 1500,
};
const STATE_RIGHT: StateSpeed = StateSpeed {
    state: CarState::Right,
    lspeed: 1500,
    rspeed: 500,
};

static SERVO_TIMER: Mutex<RefCell<Option<TIMER0>>> = Mutex::new(RefCell::new(None));
static DISPLAY: Mutex<RefCell<Option<Display<TIMER1>>>> = Mutex::new(RefCell::new(None));
static ANALOG: Mutex<RefCell<Option<Analog>>> = Mutex::new(RefCell::new(None));
static ONOFF: Mutex<RefCell<Option<bool>>> = Mutex::new(RefCell::new(None));

fn display(cstate: &CarState) {
    cortex_m::interrupt::free(|cs| {
        if let Some(display) = DISPLAY.borrow(cs).borrow_mut().as_mut() {
            match cstate {
                CarState::Stopped => display.show(&SMILE),
                CarState::Forward => display.show(&ARROW_DOWN),
                CarState::Back => display.show(&ARROW_UP),
                CarState::Left => display.show(&ARROW_LEFT),
                CarState::Right => display.show(&ARROW_RIGHT),
            }
        }
    });
}

#[entry]
fn main() -> ! {
    if let Some(board) = Board::take() {
        let display = Display::new(board.TIMER1, board.display_pins);
        let adc: Adc = Adc::new(board.ADC, AdcConfig::default_10bit());
        let anapin = board.edge.e00.into_floating_input(); // PAD0
        let analog = Analog {
            converter: adc,
            pin: anapin,
        };
        let gpiote = Gpiote::new(board.GPIOTE);
        // Servo output pins
        let servopin1 = board.edge.e01.into_push_pull_output(Level::Low).degrade(); // PAD1
        let servopin2 = board.edge.e02.into_push_pull_output(Level::Low).degrade(); // PAD2

        // Output channel for Servo 1
        gpiote
            .channel0()
            .output_pin(servopin1)
            .task_out_polarity(TaskOutPolarity::Toggle)
            .init_low();
        gpiote.channel0().task_out().write(|w| unsafe { w.bits(1) });
        // Output channel for Servo 2
        gpiote
            .channel1()
            .output_pin(servopin2)
            .task_out_polarity(TaskOutPolarity::Toggle)
            .init_low();
        gpiote.channel1().task_out().write(|w| unsafe { w.bits(1) });

        let ppi_channels = ppi::Parts::new(board.PPI);
        // Set both servo outputs high form Timer0 CC[0]
        // Set each servo output low from the respective Timer0 CC[1] and CC[2]
        // Each timer can run 3 Servos
        let mut ppi0 = ppi_channels.ppi0;
        ppi0.set_task_endpoint(gpiote.channel0().task_out());
        ppi0.set_event_endpoint(&board.TIMER0.events_compare[0]);
        ppi0.enable();
        let mut ppi1 = ppi_channels.ppi1;
        ppi1.set_task_endpoint(gpiote.channel0().task_out());
        ppi1.set_event_endpoint(&board.TIMER0.events_compare[1]);
        ppi1.enable();
        let mut ppi2 = ppi_channels.ppi2;
        ppi2.set_task_endpoint(gpiote.channel1().task_out());
        ppi2.set_event_endpoint(&board.TIMER0.events_compare[0]);
        ppi2.enable();
        let mut ppi3 = ppi_channels.ppi3;
        ppi3.set_task_endpoint(gpiote.channel1().task_out());
        ppi3.set_event_endpoint(&board.TIMER0.events_compare[2]);
        ppi3.enable();

        // The Timer PAC is used directly as the HAL does not give full access to all registers
        board.TIMER0.mode.write(|w| unsafe { w.bits(0) });
        board.TIMER0.bitmode.write(|w| unsafe { w.bits(0) });
        // CC[0] every 20 ms (50 Hz)
        board.TIMER0.cc[0].write(|w| unsafe { w.bits(20000) });
        board.TIMER0.shorts.write(|w| unsafe { w.bits(1) });
        // Servo duty cycle is from 0.5 ms to 2.5 ms with 1.5 ms for center position
        board.TIMER0.cc[1].write(|w| unsafe { w.bits(1500) });
        board.TIMER0.cc[2].write(|w| unsafe { w.bits(1500) });
        board.TIMER0.tasks_start.write(|w| unsafe { w.bits(1) });
        // Timer0 interrupt on CC[0]
        board.TIMER0.intenset.write(|w| unsafe { w.bits(1 << 16) });

        cortex_m::interrupt::free(move |cs| {
            *ONOFF.borrow(cs).borrow_mut() = Some(false);
            *SERVO_TIMER.borrow(cs).borrow_mut() = Some(board.TIMER0);
            *DISPLAY.borrow(cs).borrow_mut() = Some(display);
            *ANALOG.borrow(cs).borrow_mut() = Some(analog);
        });
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::TIMER0);
            pac::NVIC::unmask(pac::Interrupt::TIMER1);
        }

        loop {
            if let Ok(true) = board.buttons.button_a.is_low() {
                cortex_m::interrupt::free(move |cs| {
                    *ONOFF.borrow(cs).borrow_mut() = Some(true);
                });
            }
            if let Ok(true) = board.buttons.button_b.is_low() {
                cortex_m::interrupt::free(move |cs| {
                    *ONOFF.borrow(cs).borrow_mut() = Some(false);
                });
            }
        }
    }
    panic!("End");
}

#[interrupt]
fn TIMER0() {
    // Change Servo position at the start of the duty cycle. Then there is no race condition
    // between changing the duty cycle and a CC event.
    static mut STATE: StateSpeed = STATE_STOPPED;
    static mut PHOTO_CELL: i16 = 0;
    static mut IS_ON: bool = false;

    cortex_m::interrupt::free(|cs| {
        if let Some(timer) = SERVO_TIMER.borrow(cs).borrow_mut().as_mut() {
            timer.cc[1].write(|w| unsafe { w.bits(STATE.lspeed) });
            timer.cc[2].write(|w| unsafe { w.bits(STATE.rspeed) });
            timer.events_compare[0].write(|w| unsafe { w.bits(0) });
        }
        if let Some(analog) = ANALOG.borrow(cs).borrow_mut().as_mut() {
            match analog.converter.read(&mut analog.pin) {
                Ok(v) => *PHOTO_CELL = v,
                Err(_e) => *PHOTO_CELL = 0,
            };
        }
        if let Some(onoff) = ONOFF.borrow(cs).borrow().as_ref() {
            *IS_ON = *onoff;
        }
    });

    if *IS_ON {
        match PHOTO_CELL {
            i16::MIN..=64 => *STATE = STATE_LEFT,
            65..=220 => *STATE = STATE_FORWARD,
            221..=320 => *STATE = STATE_BACK,
            321..=i16::MAX => *STATE = STATE_RIGHT,
        }
    } else {
        *STATE = STATE_STOPPED;
    }
    display(&STATE.state);
}

#[interrupt]
fn TIMER1() {
    cortex_m::interrupt::free(|cs| {
        if let Some(display) = DISPLAY.borrow(cs).borrow_mut().as_mut() {
            display.handle_display_event();
        }
    });
}
