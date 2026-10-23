// SPDX-License-Identifier: BSD-3-Clause
//! The driver against a register model of the timer. The model counts at the
//! rate its configuration gives, which the tests compute from the register
//! semantics, not from the driver.

use std::cell::Cell;
use std::rc::Rc;
use std::time::Duration;

use sensor_core::bus::{Clock, Instant};
use sensor_imx_gpt::*;

/// A clock that moves 1 us each time it is read, so polling loops end.
#[derive(Default)]
struct StepClock(Cell<i64>);

impl StepClock {
    fn peek(&self) -> i64 {
        self.0.get()
    }

    fn advance(&self, d: Duration) {
        self.0.set(self.0.get() + d.as_nanos() as i64);
    }
}

impl Clock for StepClock {
    fn now(&self) -> Instant {
        self.0.set(self.0.get() + 1_000);
        Instant::from_nanos(self.0.get())
    }

    fn sleep_until(&self, t: Instant) {
        if t.as_nanos() > self.0.get() {
            self.0.set(t.as_nanos());
        }
    }
}

struct Model {
    clock: Rc<StepClock>,
    cr: u32,
    pr: u32,
    sr: u32,
    ir: u32,
    ocr1: u32,
    /// Counter value and time at the last change of rate.
    base: (u32, i64),
    /// Counter value when the compare was last checked.
    seen: u32,
    /// Whether the timer's clock is gated off.
    gated: bool,
    /// Time each read of the counter takes.
    skid: Rc<Cell<Duration>>,
    writes: Vec<(usize, u32)>,
}

impl Model {
    fn new(clock: Rc<StepClock>) -> Self {
        Model {
            clock,
            cr: 0,
            pr: 0,
            sr: 0,
            ir: 0,
            ocr1: 0,
            base: (0, 0),
            seen: 0,
            gated: false,
            skid: Rc::new(Cell::new(Duration::ZERO)),
            writes: Vec::new(),
        }
    }

    /// Counting rate from the configuration: the oscillator source is
    /// 24 MHz, divided by PRESCALER24M + 1 when EN_24M is set, then by
    /// PRESCALER + 1.
    fn hz(&self) -> u64 {
        if self.gated || self.cr & CR_EN == 0 || self.cr & CR_CLKSRC_MASK != CR_CLKSRC_OSC {
            return 0;
        }
        let mut hz = 24_000_000;
        if self.cr & CR_EN_24M != 0 {
            hz /= u64::from((self.pr >> PR_PRESCALER24M_SHIFT) & 0xf) + 1;
        }
        hz / (u64::from(self.pr & PR_PRESCALER_MASK) + 1)
    }

    fn count(&self) -> u32 {
        let (c0, t0) = self.base;
        let ticks = (self.clock.peek() - t0) as u128 * u128::from(self.hz()) / 1_000_000_000;
        c0.wrapping_add(ticks as u32)
    }

    /// Sets OF1 if the counter reached the compare value since last seen.
    fn check(&mut self) {
        let now = self.count();
        let to_compare = self.ocr1.wrapping_sub(self.seen);
        if to_compare != 0 && to_compare <= now.wrapping_sub(self.seen) {
            self.sr |= SR_OF1;
        }
        self.seen = now;
    }

    fn rebase(&mut self) {
        self.base = (self.count(), self.clock.peek());
    }
}

impl Regs for Model {
    fn read(&mut self, offset: usize) -> u32 {
        self.check();
        match offset {
            CR => self.cr,
            PR => self.pr,
            SR => self.sr,
            IR => self.ir,
            OCR1 => self.ocr1,
            CNT => {
                let c = self.count();
                self.clock.advance(self.skid.get());
                c
            }
            _ => panic!("read of offset {offset:#x}"),
        }
    }

    fn write(&mut self, offset: usize, value: u32) {
        self.check();
        self.writes.push((offset, value));
        match offset {
            CR => {
                self.rebase();
                self.cr = value;
                if value & CR_EN == 0 {
                    self.base.0 = 0;
                }
            }
            PR => {
                self.rebase();
                self.pr = value;
            }
            SR => self.sr &= !value,
            IR => self.ir = value,
            OCR1 => {
                self.ocr1 = value;
                self.seen = self.count();
            }
            _ => panic!("write of offset {offset:#x}"),
        }
    }
}

fn started() -> (Rc<StepClock>, ImxGpt<Model>) {
    let clock = Rc::new(StepClock::default());
    let mut gpt = ImxGpt::start(Model::new(clock.clone()));
    gpt.calibrate(&*clock, Duration::from_millis(1)).unwrap();
    (clock, gpt)
}

#[test]
fn start_is_linuxs_configuration_and_counts_at_3_mhz() {
    let clock = Rc::new(StepClock::default());
    let mut gpt = ImxGpt::start(Model::new(clock.clone()));
    let hz = gpt.calibrate(&*clock, Duration::from_millis(1)).unwrap();
    let model = gpt.release();
    // FRR | WAITEN | CLKSRC 5 | EN_24M | EN, and PRESCALER24M 7, as Linux's
    // imx6dl_gpt_setup_tctl writes them.
    assert_eq!(model.cr, 0x749);
    assert_eq!(model.pr, 0x7000);
    assert_eq!(model.ir, 0, "no interrupt until armed");
    assert_eq!(model.writes[0], (CR, 0), "disabled first");
    // 24 MHz / 8, within the 1 us steps of the clock over 1 ms.
    assert!((2_990_000..=3_010_000).contains(&hz), "{hz} Hz");
}

#[test]
fn a_counter_that_does_not_move_does_not_calibrate() {
    let clock = Rc::new(StepClock::default());
    let mut model = Model::new(clock.clone());
    model.gated = true;
    let mut gpt = ImxGpt::start(model);
    assert_eq!(gpt.calibrate(&*clock, Duration::from_millis(1)), None);
}

#[test]
fn ticks_round_up_and_stay_within_half_the_range() {
    let (_, gpt) = started();
    let hz = gpt.hz();
    assert_eq!(gpt.ticks_for(Duration::ZERO), 1);
    assert_eq!(gpt.ticks_for(Duration::from_nanos(1)), 1);
    let ms = gpt.ticks_for(Duration::from_millis(1));
    assert_eq!(u64::from(ms), hz.div_ceil(1000));
    assert_eq!(
        gpt.ticks_for(Duration::from_secs(1_000_000)),
        i32::MAX as u32
    );
}

#[test]
fn an_armed_compare_expires_once_after_its_ticks() {
    let (clock, mut gpt) = started();
    let ticks = gpt.ticks_for(Duration::from_millis(2));
    assert_eq!(gpt.arm(ticks), Arm::Armed);
    clock.advance(Duration::from_micros(1_900));
    assert!(!gpt.take_expiry(), "not yet");
    clock.advance(Duration::from_micros(200));
    assert!(gpt.take_expiry());
    assert!(!gpt.take_expiry(), "only once");
    assert_eq!(gpt.release().ir, 0, "disarmed after expiry");
}

#[test]
fn a_compare_the_counter_passed_while_it_was_set_is_reported() {
    let clock = Rc::new(StepClock::default());
    let model = Model::new(clock.clone());
    // Each read of the counter takes 10 us, 30 ticks: far more than the 1
    // tick asked for.
    model.skid.set(Duration::from_micros(10));
    let mut gpt = ImxGpt::start(model);
    assert_eq!(gpt.arm(1), Arm::Passed);
    clock.advance(Duration::from_millis(1));
    assert!(!gpt.take_expiry());
    assert_eq!(gpt.release().ir, 0, "disarmed");
}

#[test]
fn a_disarmed_compare_never_expires() {
    let (clock, mut gpt) = started();
    let ticks = gpt.ticks_for(Duration::from_millis(1));
    gpt.arm(ticks);
    gpt.disarm();
    clock.advance(Duration::from_millis(5));
    assert!(!gpt.take_expiry());
    assert_eq!(gpt.release().ir, 0);
}

#[test]
fn a_compare_across_the_counters_wrap_expires() {
    let (clock, mut gpt) = started();
    // Run the counter to just short of its wrap: 2^32 ticks at 3 MHz is
    // about 1432 s.
    let to_wrap = u64::from(u32::MAX - gpt.count()) * 1_000_000_000 / gpt.hz();
    clock.advance(Duration::from_nanos(to_wrap) - Duration::from_micros(10));
    let ticks = gpt.ticks_for(Duration::from_millis(1));
    assert_eq!(gpt.arm(ticks), Arm::Armed);
    clock.advance(Duration::from_micros(500));
    assert!(!gpt.take_expiry(), "the wrap alone is not a match");
    clock.advance(Duration::from_micros(600));
    assert!(gpt.take_expiry());
}

#[test]
fn a_match_of_the_old_compare_does_not_expire_the_new_one() {
    let clock = Rc::new(StepClock::default());
    let model = Model::new(clock.clone());
    let skid = model.skid.clone();
    let mut gpt = ImxGpt::start(model);
    gpt.calibrate(&*clock, Duration::from_millis(1)).unwrap();
    let ticks = gpt.ticks_for(Duration::from_millis(1));
    assert_eq!(gpt.arm(ticks), Arm::Armed);
    // Re-armed 5 us before the old compare, with a counter read that takes
    // 10 us: the old value matches while the new one is being set.
    clock.advance(Duration::from_micros(995));
    skid.set(Duration::from_micros(10));
    let ticks = gpt.ticks_for(Duration::from_millis(5));
    assert_eq!(gpt.arm(ticks), Arm::Armed);
    skid.set(Duration::ZERO);
    assert!(
        !gpt.take_expiry(),
        "the old compare's match is not this one"
    );
    clock.advance(Duration::from_millis(5));
    assert!(gpt.take_expiry());
}
