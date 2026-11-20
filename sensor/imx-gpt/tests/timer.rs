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
    ocr: [u32; 3],
    /// Counter value and time at the last change of rate.
    base: (u32, i64),
    /// Counter value when each compare was last checked.
    seen: [u32; 3],
    /// Whether the timer's clock is gated off.
    gated: bool,
    /// Time each read of the counter takes, after it samples the counter.
    skid: Rc<Cell<Duration>>,
    /// Time each register write takes, after it lands: the counter moves on,
    /// and a compare can match, between one write and the next.
    write_time: Rc<Cell<Duration>>,
    /// How long the first read after a register write waits for the write,
    /// before it samples: the count it returns is that much late.
    write_stall: Duration,
    /// Whether a register write has happened since the last read.
    stall_pending: bool,
    /// What IR was last written as, readable after the model has moved.
    ir_out: Rc<Cell<u32>>,
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
            ocr: [0; 3],
            base: (0, 0),
            seen: [0; 3],
            gated: false,
            skid: Rc::new(Cell::new(Duration::ZERO)),
            write_time: Rc::new(Cell::new(Duration::ZERO)),
            write_stall: Duration::ZERO,
            stall_pending: false,
            ir_out: Rc::new(Cell::new(0)),
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

    /// Sets each compare's flag if the counter reached its value since last
    /// seen. The flags set whether or not their interrupts are enabled.
    fn check(&mut self) {
        let now = self.count();
        for c in Compare::ALL {
            let i = index(c);
            let to_compare = self.ocr[i].wrapping_sub(self.seen[i]);
            if to_compare != 0 && to_compare <= now.wrapping_sub(self.seen[i]) {
                self.sr |= c.flag();
            }
            self.seen[i] = now;
        }
    }

    fn rebase(&mut self) {
        self.base = (self.count(), self.clock.peek());
    }
}

impl Regs for Model {
    fn read(&mut self, offset: usize) -> u32 {
        if std::mem::take(&mut self.stall_pending) {
            self.clock.advance(self.write_stall);
        }
        self.check();
        match offset {
            CR => self.cr,
            PR => self.pr,
            SR => self.sr,
            IR => self.ir,
            OCR1 => self.ocr[0],
            OCR2 => self.ocr[1],
            OCR3 => self.ocr[2],
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
        self.stall_pending = true;
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
            IR => {
                self.ir = value;
                self.ir_out.set(value);
            }
            OCR1 | OCR2 | OCR3 => {
                let i = (offset - OCR1) / 4;
                self.ocr[i] = value;
                self.seen[i] = self.count();
            }
            _ => panic!("write of offset {offset:#x}"),
        }
        self.clock.advance(self.write_time.get());
    }
}

fn index(c: Compare) -> usize {
    match c {
        Compare::One => 0,
        Compare::Two => 1,
        Compare::Three => 2,
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
    assert_eq!(gpt.arm(Compare::One, ticks), Arm::Armed);
    clock.advance(Duration::from_micros(1_900));
    assert!(!gpt.take_expiry(Compare::One), "not yet");
    clock.advance(Duration::from_micros(200));
    assert!(gpt.take_expiry(Compare::One));
    assert!(!gpt.take_expiry(Compare::One), "only once");
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
    assert_eq!(gpt.arm(Compare::One, 1), Arm::Passed);
    clock.advance(Duration::from_millis(1));
    assert!(!gpt.take_expiry(Compare::One));
    assert_eq!(gpt.release().ir, 0, "disarmed");
}

#[test]
fn a_disarmed_compare_never_expires() {
    let (clock, mut gpt) = started();
    let ticks = gpt.ticks_for(Duration::from_millis(1));
    gpt.arm(Compare::One, ticks);
    gpt.disarm(Compare::One);
    clock.advance(Duration::from_millis(5));
    assert!(!gpt.take_expiry(Compare::One));
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
    assert_eq!(gpt.arm(Compare::One, ticks), Arm::Armed);
    clock.advance(Duration::from_micros(500));
    assert!(
        !gpt.take_expiry(Compare::One),
        "the wrap alone is not a match"
    );
    clock.advance(Duration::from_micros(600));
    assert!(gpt.take_expiry(Compare::One));
}

#[test]
fn a_match_of_the_old_compare_does_not_expire_the_new_one() {
    // Re-armed at every lead from 0 to 40 us before the old compare, once
    // with each register write taking 10 us and once with each counter read
    // taking 10 us, so that for some leads the old value matches while the
    // new one is being set: between two of the writes, or during the read.
    let slow_writes = (Duration::ZERO, Duration::from_micros(10));
    let slow_reads = (Duration::from_micros(10), Duration::ZERO);
    for (read, write) in [slow_writes, slow_reads] {
        for lead in 0..=40 {
            let clock = Rc::new(StepClock::default());
            let model = Model::new(clock.clone());
            let (skid, write_time) = (model.skid.clone(), model.write_time.clone());
            let mut gpt = ImxGpt::start(model);
            gpt.calibrate(&*clock, Duration::from_millis(1)).unwrap();
            let ticks = gpt.ticks_for(Duration::from_millis(1));
            assert_eq!(gpt.arm(Compare::One, ticks), Arm::Armed);
            clock.advance(Duration::from_micros(1_000 - lead));
            skid.set(read);
            write_time.set(write);
            let ticks = gpt.ticks_for(Duration::from_millis(5));
            assert_eq!(gpt.arm(Compare::One, ticks), Arm::Armed);
            skid.set(Duration::ZERO);
            write_time.set(Duration::ZERO);
            assert!(
                !gpt.take_expiry(Compare::One),
                "the old compare's match passed for the new one: re-armed \
                 {lead} us before it, reads {read:?}, writes {write:?}"
            );
            clock.advance(Duration::from_millis(5));
            assert!(gpt.take_expiry(Compare::One), "lead {lead} us");
        }
    }
}

#[test]
fn a_compare_the_counter_has_reached_as_it_is_set_is_reported() {
    // QEMU's timer matches a compare only while the counter is below it
    // (imx_gpt_find_limit: count < reg), and this model likewise: a compare
    // the counter has already reached when it is written does not match
    // until the counter wraps. The driver must say so, not wait for it.
    let clock = Rc::new(StepClock::default());
    let model = Model::new(clock.clone());
    let skid = model.skid.clone();
    let mut gpt = ImxGpt::start(model);
    // At 3 MHz the counter steps every 333.3 ns: 1 ms and 200 ns after the
    // start it reads 3000, 0.6 of a tick in.
    clock.advance(Duration::from_nanos(1_000_200));
    // The counter read takes 200 ns, so by the time the compare is written,
    // one tick past the value read, the counter is on it, and it is still
    // there when the driver looks again: 0 ticks ahead, nothing matched.
    skid.set(Duration::from_nanos(200));
    assert_eq!(gpt.arm(Compare::One, 1), Arm::Passed);
    skid.set(Duration::ZERO);
    clock.advance(Duration::from_millis(1));
    assert!(!gpt.take_expiry(Compare::One));
    assert_eq!(gpt.release().ir, 0, "disarmed");
}

#[test]
fn the_compare_registers_and_flags_are_the_timers() {
    // As QEMU's model of the timer (hw/timer/imx_gpt.c) has them: OCR1 to
    // OCR3 are the fifth to seventh words, and each compare's status flag
    // and interrupt enable are bits 0 to 2.
    assert_eq!(
        Compare::ALL.map(|c| (c.ocr(), c.flag())),
        [(0x10, 1 << 0), (0x14, 1 << 1), (0x18, 1 << 2)]
    );
    assert_eq!((IR_OF1IE, IR_OF2IE, IR_OF3IE), (SR_OF1, SR_OF2, SR_OF3));
}

#[test]
fn every_compare_expires_on_its_own_register_and_flag() {
    for c in Compare::ALL {
        let (clock, mut gpt) = started();
        let ticks = gpt.ticks_for(Duration::from_millis(1));
        assert_eq!(gpt.arm(c, ticks), Arm::Armed);
        clock.advance(Duration::from_micros(1_100));
        for other in Compare::ALL.into_iter().filter(|&o| o != c) {
            assert!(!gpt.take_expiry(other), "{other:?}, with only {c:?} armed");
        }
        assert!(gpt.take_expiry(c), "{c:?}");
        let model = gpt.release();
        assert!(model.writes.iter().any(|&(o, _)| o == c.ocr()), "{c:?}");
        assert_eq!(model.ir, 0, "{c:?} disarmed after expiry");
    }
}

#[test]
fn two_compares_expire_apart() {
    let (clock, mut gpt) = started();
    let two_ms = gpt.ticks_for(Duration::from_millis(2));
    let five_ms = gpt.ticks_for(Duration::from_millis(5));
    assert_eq!(gpt.arm(Compare::One, two_ms), Arm::Armed);
    assert_eq!(gpt.arm(Compare::Two, five_ms), Arm::Armed);
    clock.advance(Duration::from_micros(2_100));
    assert!(
        !gpt.take_expiry(Compare::Two),
        "the first compare's match is not the second's"
    );
    assert!(gpt.take_expiry(Compare::One));
    clock.advance(Duration::from_micros(2_700));
    assert!(!gpt.take_expiry(Compare::Two), "not yet");
    clock.advance(Duration::from_micros(300));
    assert!(gpt.take_expiry(Compare::Two));
    assert!(!gpt.take_expiry(Compare::One), "the first only once");
    assert_eq!(gpt.release().ir, 0);
}

#[test]
fn arming_and_disarming_one_compare_keeps_the_others_interrupt() {
    let clock = Rc::new(StepClock::default());
    let model = Model::new(clock.clone());
    let ir = model.ir_out.clone();
    let mut gpt = ImxGpt::start(model);
    gpt.calibrate(&*clock, Duration::from_millis(1)).unwrap();
    let three_ms = gpt.ticks_for(Duration::from_millis(3));
    assert_eq!(gpt.arm(Compare::Two, three_ms), Arm::Armed);
    let one_ms = gpt.ticks_for(Duration::from_millis(1));
    assert_eq!(gpt.arm(Compare::One, one_ms), Arm::Armed);
    assert_eq!(ir.get(), IR_OF1IE | IR_OF2IE);
    gpt.disarm(Compare::One);
    assert_eq!(
        ir.get(),
        IR_OF2IE,
        "the second's interrupt is still enabled"
    );
    assert_eq!(gpt.arm(Compare::One, one_ms), Arm::Armed);
    clock.advance(Duration::from_micros(1_100));
    assert!(gpt.take_expiry(Compare::One));
    assert_eq!(ir.get(), IR_OF2IE, "nor did the first's expiry disable it");
    clock.advance(Duration::from_millis(2));
    assert!(gpt.take_expiry(Compare::Two));
    assert_eq!(ir.get(), 0);
}

#[test]
fn a_match_waiting_to_be_taken_survives_another_compare_being_set() {
    let (clock, mut gpt) = started();
    let ticks = gpt.ticks_for(Duration::from_millis(1));
    assert_eq!(gpt.arm(Compare::Two, ticks), Arm::Armed);
    clock.advance(Duration::from_micros(1_100));
    // The second has matched, and the interrupt is not yet handled.
    assert_eq!(gpt.arm(Compare::One, ticks), Arm::Armed);
    gpt.disarm(Compare::One);
    assert!(gpt.take_expiry(Compare::Two), "its flag was not cleared");
}

#[test]
fn the_time_a_counter_read_takes_does_not_skew_the_rate() {
    let clock = Rc::new(StepClock::default());
    let model = Model::new(clock.clone());
    // Each read of the counter takes 10 us, 1 % of the measurement: counted
    // into the interval, it would measure about 2.97 MHz.
    model.skid.set(Duration::from_micros(10));
    let mut gpt = ImxGpt::start(model);
    let hz = gpt.calibrate(&*clock, Duration::from_millis(1)).unwrap();
    // 24 MHz / 8, within a tick of the 3000 counted.
    assert!((2_999_000..=3_001_000).contains(&hz), "{hz} Hz");
}

#[test]
fn a_read_held_up_by_the_writes_before_it_does_not_skew_the_rate() {
    // The first read after register writes waits for them and samples the
    // counter at the end of the wait: on the board about 1.4 us, here 10 us.
    // start() writes just before calibrate reads; a first count read then
    // would sit 5 us, half the wait, after the midpoint of its clock readings
    // and measure the rate about 313 ppm low over 16 ms (2999062 Hz).
    let clock = Rc::new(StepClock::default());
    let mut model = Model::new(clock.clone());
    model.write_stall = Duration::from_micros(10);
    let mut gpt = ImxGpt::start(model);
    let hz = gpt.calibrate(&*clock, Duration::from_millis(16)).unwrap();
    // 24 MHz / 8, within a tick, 62.5 Hz over 16 ms.
    assert!(hz.abs_diff(3_000_000) <= 62, "{hz} Hz");
}
