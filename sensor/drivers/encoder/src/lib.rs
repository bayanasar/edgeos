// SPDX-License-Identifier: BSD-3-Clause
//! Incremental rotary encoder with a push switch (the KY-040 class module:
//! CLK, DT, SW), on three GPIO lines.
//!
//! CLK and DT are the encoder's A and B contacts, a 2-bit quadrature (Gray)
//! code: turning the shaft changes one contact at a time, and the order says
//! the direction. Contact bounce is up to 2 ms at 15 RPM on a representative
//! 12 mm encoder (Bourns PEC11R data sheet); the part on the kit's module is
//! not identified. SW closes to ground while the shaft is pressed.
//!
//! # Decoding
//!
//! Each edge event changes one contact. The tracked state moves one step
//! round the Gray cycle, forward or back; bounce on one contact while the
//! other is still produces steps that cancel. A detent is reported only when
//! the state reaches a rest position having moved a whole detent's worth of
//! steps in one direction, so an incomplete turn or a bounce at the detent
//! counts nothing. Which state is a rest position comes from the levels read
//! at [`Encoder::init`], when the shaft sits in a detent.
//!
//! With one event per edge a two-contact jump cannot arrive at once. Its
//! equivalent is an edge reporting the level its line already had, which
//! means an edge of that line was lost (bounce faster than the transport
//! reports, for instance). Such an edge is rejected and counted, and the
//! state stays where it was; a contact that bounced back ends at the level
//! the next edge reports. A transport overflow resynchronises from the
//! lines' levels, and a two-contact difference found then is also counted.
//!
//! Clockwise is CLK changing before DT, as in common KY-040 example code; the
//! encoder's own data sheet is not available. Check the sign on hardware.
//!
//! # Output
//!
//! The encoder produces [`Event`]s, not [`Sample`](sensor_core::Sample)s: a
//! sample is a measurement taken on request, and a detent or a press is a
//! discrete occurrence, which is what an event records. A detent is
//! [`EventType::Delta`] with value +1 (clockwise) or -1, timestamped at the
//! edge that completed it, and a switch change is [`EventType::State`] with 1
//! for pressed and 0 for released. One event per occurrence keeps each one's
//! own timestamp and lets a consumer detect loss from the sequence number;
//! a position is the sum of the deltas, which the consumer keeps if it wants
//! one.
//!
//! The switch is debounced by lockout: the first edge that changes its state
//! is reported at once with that edge's time, further switch edges are
//! ignored for [`SWITCH_LOCKOUT`], and then the level is read back and a
//! change that happened during the lockout is reported with the read-back
//! time. The lockout length is an assumption, not a data-sheet value: the
//! switch's bounce is not specified.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::time::Duration;

use sensor_core::bus::{
    Bias, BusError, Clock, Direction, Edges, Gpio, GpioEvent, Instant, LineConfig, TsSource,
};
use sensor_core::sample::{Event, EventType, Flags};

/// How long switch edges are ignored after a reported change. Not from a
/// data sheet; see the crate documentation.
pub const SWITCH_LOCKOUT: Duration = Duration::from_millis(10);

/// Edges read from the transport at a time; each makes at most one event.
const BATCH: usize = 16;

/// Quadrature steps per detent: four for one full cycle per detent (equal
/// detent and pulse counts, the common KY-040 part), two for two detents per
/// cycle, one to report every step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Steps {
    One = 1,
    Two = 2,
    Four = 4,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pins {
    pub clk: u32,
    pub dt: u32,
    pub sw: Option<u32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Bus(BusError),
    /// The clock cannot represent a deadline.
    Clock,
}

impl From<BusError> for Error {
    fn from(e: BusError) -> Self {
        Error::Bus(e)
    }
}

/// Position of a CLK/DT state (CLK in bit 1) on the clockwise cycle
/// 11, 01, 00, 10.
const CYCLE_INDEX: [u8; 4] = [2, 1, 3, 0];

/// Steps from `from` to `to`: +1 clockwise, -1 counter-clockwise, 0 for no
/// change, `None` if both contacts changed.
fn step(from: u8, to: u8) -> Option<i8> {
    match (CYCLE_INDEX[usize::from(to)] + 4 - CYCLE_INDEX[usize::from(from)]) % 4 {
        0 => Some(0),
        1 => Some(1),
        3 => Some(-1),
        _ => None,
    }
}

/// A fixed-capacity queue of events not yet handed out.
struct Queue {
    buf: [Event; BATCH],
    head: usize,
    len: usize,
}

impl Queue {
    const fn new() -> Self {
        Queue {
            buf: [Event {
                timestamp: 0,
                sensor_id: 0,
                seq: 0,
                epoch: 0,
                event_type: 0,
                flags: 0,
                value: 0,
                ts_source: 0,
                reserved: [0; 3],
            }; BATCH],
            head: 0,
            len: 0,
        }
    }

    fn push(&mut self, e: Event) {
        // Callers fill the queue only from one batch, after it was empty.
        debug_assert!(self.len < BATCH);
        self.buf[(self.head + self.len) % BATCH] = e;
        self.len += 1;
    }

    fn pop(&mut self) -> Option<Event> {
        if self.len == 0 {
            return None;
        }
        let e = self.buf[self.head];
        self.head = (self.head + 1) % BATCH;
        self.len -= 1;
        Some(e)
    }
}

pub struct Encoder<G> {
    gpio: G,
    pins: Pins,
    steps: Steps,
    sensor_id: u32,
    epoch: u32,
    seq: u32,
    /// CLK in bit 1, DT in bit 0, as last tracked.
    state: u8,
    /// Cycle index of a rest position, modulo `steps`.
    rest: u8,
    /// Steps since the last rest position.
    moved: i8,
    pressed: bool,
    /// While set, switch edges are ignored until this time, when the level is
    /// read back.
    lockout_until: Option<Instant>,
    rejected: u32,
    queue: Queue,
    /// Flags for the next event handed out.
    pending_flags: Flags,
    initialized: bool,
}

const INPUT: LineConfig = LineConfig {
    direction: Direction::Input,
    bias: Bias::PullUp,
    edges: Edges::BOTH,
};

impl<G: Gpio> Encoder<G> {
    /// No line is touched until [`Encoder::init`].
    pub fn new(gpio: G, pins: Pins, steps: Steps, sensor_id: u32, epoch: u32) -> Self {
        Encoder {
            gpio,
            pins,
            steps,
            sensor_id,
            epoch,
            seq: 0,
            state: 0,
            rest: 0,
            moved: 0,
            pressed: false,
            lockout_until: None,
            rejected: 0,
            queue: Queue::new(),
            pending_flags: Flags::NEW_EPOCH,
            initialized: false,
        }
    }

    pub fn release(self) -> G {
        self.gpio
    }

    /// Edges rejected so far because they did not move the tracked state,
    /// and two-contact jumps found on resynchronising.
    pub fn rejected(&self) -> u32 {
        self.rejected
    }

    /// All lines as inputs with pull-ups, reporting both edges. The module
    /// has pull-ups on CLK and DT; SW often has none. Must run with the shaft
    /// at rest in a detent, since that position defines the detents.
    pub fn init(&mut self) -> Result<(), Error> {
        self.gpio.configure(self.pins.clk, INPUT)?;
        self.gpio.configure(self.pins.dt, INPUT)?;
        if let Some(sw) = self.pins.sw {
            self.gpio.configure(sw, INPUT)?;
        }
        self.state = self.levels()?;
        self.rest = CYCLE_INDEX[usize::from(self.state)] % self.steps as u8;
        self.moved = 0;
        if let Some(sw) = self.pins.sw {
            self.pressed = !self.gpio.get(sw)?;
        }
        self.lockout_until = None;
        self.initialized = true;
        Ok(())
    }

    /// The next detent or switch change, or `None` if there is none by
    /// `deadline`.
    pub fn next_event<C: Clock>(
        &mut self,
        clock: &C,
        deadline: Instant,
    ) -> Result<Option<Event>, Error> {
        if !self.initialized {
            self.init()?;
        }
        let mut buf = [GpioEvent::default(); BATCH];
        loop {
            if let Some(e) = self.queue.pop() {
                return Ok(Some(e));
            }
            if self.lockout_until.is_some_and(|t| clock.now() >= t) {
                self.end_lockout(clock)?;
                continue;
            }
            let until = match self.lockout_until {
                Some(t) if t < deadline => t,
                _ => deadline,
            };
            match self.gpio.wait_edges(&mut buf, until) {
                Ok(n) => {
                    for e in &buf[..n] {
                        self.edge(e);
                    }
                }
                Err(BusError::Timeout) if until < deadline => {}
                Err(BusError::Timeout) => return Ok(None),
                Err(BusError::Overflow) => self.resync(clock)?,
                Err(e) => return Err(e.into()),
            }
        }
    }

    fn levels(&mut self) -> Result<u8, Error> {
        let clk = self.gpio.get(self.pins.clk)?;
        let dt = self.gpio.get(self.pins.dt)?;
        Ok((u8::from(clk) << 1) | u8::from(dt))
    }

    fn edge(&mut self, e: &GpioEvent) {
        let high = e.level != 0;
        if Some(e.line) == self.pins.sw {
            let pressed = !high;
            if self.lockout_until.is_none() && pressed != self.pressed {
                self.pressed = pressed;
                let t = Instant::from_nanos(e.timestamp);
                self.lockout_until = t.checked_add(SWITCH_LOCKOUT);
                self.emit(EventType::State, i32::from(pressed), e.timestamp, e.ts_source);
            }
            return;
        }
        let bit = if e.line == self.pins.clk {
            1 << 1
        } else if e.line == self.pins.dt {
            1
        } else {
            return;
        };
        let next = if high { self.state | bit } else { self.state & !bit };
        let s = match step(self.state, next) {
            Some(0) | None => {
                self.rejected = self.rejected.wrapping_add(1);
                return;
            }
            Some(s) => s,
        };
        self.state = next;
        self.moved += s;
        let steps = self.steps as u8;
        if CYCLE_INDEX[usize::from(next)] % steps == self.rest {
            let whole = steps as i8;
            if self.moved == whole || self.moved == -whole {
                self.emit(
                    EventType::Delta,
                    i32::from(self.moved.signum()),
                    e.timestamp,
                    e.ts_source,
                );
            }
            self.moved = 0;
        }
    }

    /// Reads the switch back at the end of a lockout.
    fn end_lockout<C: Clock>(&mut self, clock: &C) -> Result<(), Error> {
        self.lockout_until = None;
        let Some(sw) = self.pins.sw else {
            return Ok(());
        };
        let pressed = !self.gpio.get(sw)?;
        if pressed != self.pressed {
            self.pressed = pressed;
            let now = clock.now();
            self.lockout_until = now.checked_add(SWITCH_LOCKOUT);
            self.emit(
                EventType::State,
                i32::from(pressed),
                now.as_nanos(),
                TsSource::DriverRead as u8,
            );
        }
        Ok(())
    }

    /// After lost edges: take the state from the lines' levels, drop the
    /// partial detent, and mark the next event.
    fn resync<C: Clock>(&mut self, clock: &C) -> Result<(), Error> {
        let now = self.levels()?;
        if step(self.state, now).is_none() {
            self.rejected = self.rejected.wrapping_add(1);
        }
        self.state = now;
        self.moved = 0;
        self.pending_flags = self.pending_flags.union(Flags::SEQ_GAP);
        if self.pins.sw.is_some() {
            // A switch change lost with the edges is found by reading back.
            self.lockout_until = Some(clock.now());
        }
        Ok(())
    }

    fn emit(&mut self, kind: EventType, value: i32, timestamp: i64, ts_source: u8) {
        self.queue.push(Event {
            timestamp,
            sensor_id: self.sensor_id,
            seq: self.seq,
            epoch: self.epoch,
            event_type: kind as u16,
            flags: self.pending_flags.0,
            value,
            ts_source,
            reserved: [0; 3],
        });
        self.pending_flags = Flags::default();
        self.seq = self.seq.wrapping_add(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sensor_fake::{FakeClock, FakeGpio};
    use std::rc::Rc;

    const CLK: u32 = 17;
    const DT: u32 = 27;
    const SW: u32 = 22;
    const PINS: Pins = Pins {
        clk: CLK,
        dt: DT,
        sw: Some(SW),
    };

    /// Contact levels (CLK, DT) of one clockwise detent from the rest
    /// position, both high, independently of the driver's table.
    const CW: [(bool, bool); 4] = [(false, true), (false, false), (true, false), (true, true)];

    struct Rig {
        clock: Rc<FakeClock>,
        enc: Encoder<FakeGpio>,
        /// Time (ns from start) for the next scheduled edge.
        t: u64,
    }

    impl Rig {
        fn new(steps: Steps) -> Self {
            let clock = Rc::new(FakeClock::new());
            let mut g = FakeGpio::new(Rc::clone(&clock));
            // Pulled up, shaft at rest, switch open.
            for line in [CLK, DT, SW] {
                g.configure(line, INPUT).unwrap();
                g.lines.get_mut(&line).unwrap().level = true;
            }
            let mut enc = Encoder::new(g, PINS, steps, 9, 1);
            enc.init().unwrap();
            Rig { clock, enc, t: 0 }
        }

        /// Schedules `level` on `line` `gap_us` after the previous edge, or
        /// after now if the clock has passed it.
        fn edge(&mut self, line: u32, level: bool, gap_us: u64) {
            let now = self.clock.now().as_nanos() as u64;
            self.t = self.t.max(now) + gap_us * 1_000;
            let after = Duration::from_nanos(self.t - now);
            self.enc.gpio.schedule(line, level, after);
        }

        /// Schedules the contact levels of `detents` turns, 500 us apart.
        fn turn(&mut self, detents: i32) {
            for _ in 0..detents.unsigned_abs() {
                let seq: Vec<_> = if detents > 0 {
                    CW.to_vec()
                } else {
                    // Counter-clockwise visits the same states backwards.
                    vec![(true, false), (false, false), (false, true), (true, true)]
                };
                let mut prev = (true, true);
                for (clk, dt) in seq {
                    if clk != prev.0 {
                        self.edge(CLK, clk, 500);
                    }
                    if dt != prev.1 {
                        self.edge(DT, dt, 500);
                    }
                    prev = (clk, dt);
                }
            }
        }

        /// Every event until the queue of scheduled edges runs dry.
        fn events(&mut self) -> Vec<Event> {
            let mut out = Vec::new();
            loop {
                let deadline = self.clock.now().checked_add(Duration::from_secs(1)).unwrap();
                match self.enc.next_event(&*self.clock, deadline).unwrap() {
                    Some(e) => out.push(e),
                    None => return out,
                }
            }
        }
    }

    fn deltas(ev: &[Event]) -> Vec<i32> {
        ev.iter()
            .filter(|e| e.event_type() == Some(EventType::Delta))
            .map(|e| e.value)
            .collect()
    }

    #[test]
    fn the_step_table_is_the_gray_cycle() {
        // Clockwise: 11 -> 01 -> 00 -> 10 -> 11.
        let cw = [0b11, 0b01, 0b00, 0b10, 0b11];
        for w in cw.windows(2) {
            assert_eq!(step(w[0], w[1]), Some(1), "{:02b}->{:02b}", w[0], w[1]);
            assert_eq!(step(w[1], w[0]), Some(-1));
        }
        for s in 0..4 {
            assert_eq!(step(s, s), Some(0));
            assert_eq!(step(s, s ^ 0b11), None, "both contacts changed");
        }
    }

    #[test]
    fn whole_detents_count_in_both_directions() {
        let mut r = Rig::new(Steps::Four);
        r.turn(3);
        r.turn(-2);
        let ev = r.events();
        assert_eq!(deltas(&ev), [1, 1, 1, -1, -1]);
        assert_eq!(r.enc.rejected(), 0);
        // Each detent is stamped at the edge that completed it: the fourth
        // edge of each, 500 us apart.
        assert_eq!(ev[0].timestamp, 2_000_000);
        assert_eq!(ev[1].timestamp, 4_000_000);
        assert_eq!(ev[0].ts_source, TsSource::Interrupt as u8);
    }

    #[test]
    fn events_carry_sequence_epoch_and_the_first_is_marked() {
        let mut r = Rig::new(Steps::Four);
        r.turn(2);
        let ev = r.events();
        assert_eq!((ev[0].seq, ev[1].seq, ev[0].epoch, ev[0].sensor_id), (0, 1, 1, 9));
        assert!(ev[0].flags().contains(Flags::NEW_EPOCH));
        assert!(!ev[1].flags().contains(Flags::NEW_EPOCH));
    }

    #[test]
    fn bounce_on_one_contact_cancels() {
        let mut r = Rig::new(Steps::Four);
        // CLK chatters three times before settling low, then the detent
        // completes normally.
        for _ in 0..3 {
            r.edge(CLK, false, 100);
            r.edge(CLK, true, 100);
        }
        r.turn(1);
        assert_eq!(deltas(&r.events()), [1]);
        assert_eq!(r.enc.rejected(), 0);
    }

    #[test]
    fn a_turn_abandoned_halfway_counts_nothing() {
        let mut r = Rig::new(Steps::Four);
        // Two steps clockwise, then back to the same detent.
        r.edge(CLK, false, 500);
        r.edge(DT, false, 500);
        r.edge(DT, true, 500);
        r.edge(CLK, true, 500);
        r.turn(-1);
        assert_eq!(deltas(&r.events()), [-1]);
    }

    #[test]
    fn bounce_at_the_detent_does_not_count_twice() {
        let mut r = Rig::new(Steps::Four);
        r.turn(1);
        // DT, the contact that completed the detent, chatters.
        r.edge(DT, false, 50);
        r.edge(DT, true, 50);
        r.edge(DT, false, 50);
        r.edge(DT, true, 50);
        assert_eq!(deltas(&r.events()), [1]);
    }

    #[test]
    fn an_edge_that_does_not_change_its_line_is_rejected() {
        let mut r = Rig::new(Steps::Four);
        // The fake reports only changes, so inject the duplicate directly:
        // a falling CLK lost, then CLK reported high again.
        let dup = GpioEvent {
            timestamp: 100,
            line: CLK,
            level: 1,
            ts_source: TsSource::Interrupt as u8,
            reserved: 0,
        };
        r.enc.edge(&dup);
        assert_eq!(r.enc.rejected(), 1);
        r.turn(1);
        assert_eq!(deltas(&r.events()), [1]);
    }

    #[test]
    fn half_and_single_step_detents() {
        let mut r = Rig::new(Steps::Two);
        r.turn(1); // one full cycle is two detents
        r.turn(-1);
        assert_eq!(deltas(&r.events()), [1, 1, -1, -1]);
        let mut r = Rig::new(Steps::One);
        r.turn(1);
        assert_eq!(deltas(&r.events()), [1, 1, 1, 1]);
    }

    #[test]
    fn rest_comes_from_the_levels_at_init() {
        // A part whose detent sits with both contacts low.
        let clock = Rc::new(FakeClock::new());
        let mut g = FakeGpio::new(Rc::clone(&clock));
        for line in [CLK, DT, SW] {
            g.configure(line, INPUT).unwrap();
            g.lines.get_mut(&line).unwrap().level = line == SW;
        }
        let mut enc = Encoder::new(g, PINS, Steps::Four, 9, 1);
        enc.init().unwrap();
        // Clockwise from 00: 10, 11, 01, 00.
        for (i, (line, level)) in [(CLK, true), (DT, true), (CLK, false), (DT, false)]
            .into_iter()
            .enumerate()
        {
            enc.gpio
                .schedule(line, level, Duration::from_micros(500 * (i as u64 + 1)));
        }
        let deadline = clock.now().checked_add(Duration::from_secs(1)).unwrap();
        let e = enc.next_event(&*clock, deadline).unwrap().unwrap();
        assert_eq!((e.value, e.timestamp), (1, 2_000_000));
        assert_eq!(enc.next_event(&*clock, deadline), Ok(None));
    }

    #[test]
    fn the_switch_reports_press_and_release_once_through_bounce() {
        let mut r = Rig::new(Steps::Four);
        r.edge(SW, false, 1_000); // pressed at 1 ms
        r.edge(SW, true, 200);
        r.edge(SW, false, 200);
        r.edge(SW, true, 50_000); // released at 51.4 ms
        r.edge(SW, false, 300);
        r.edge(SW, true, 300);
        let ev = r.events();
        let sw: Vec<_> = ev
            .iter()
            .map(|e| (e.event_type(), e.value, e.timestamp))
            .collect();
        assert_eq!(
            sw,
            [
                (Some(EventType::State), 1, 1_000_000),
                (Some(EventType::State), 0, 51_400_000),
            ]
        );
    }

    #[test]
    fn a_release_inside_the_lockout_is_found_by_reading_back() {
        let mut r = Rig::new(Steps::Four);
        r.edge(SW, false, 1_000); // pressed at 1 ms
        r.edge(SW, true, 3_000); // released at 4 ms, inside the lockout
        let ev = r.events();
        assert_eq!(ev.len(), 2);
        assert_eq!((ev[0].value, ev[0].timestamp), (1, 1_000_000));
        // Read back when the lockout ends, 10 ms after the press.
        assert_eq!((ev[1].value, ev[1].timestamp), (0, 11_000_000));
        assert_eq!(ev[1].ts_source, TsSource::DriverRead as u8);
    }

    #[test]
    fn no_switch_line_is_fine() {
        let clock = Rc::new(FakeClock::new());
        let mut g = FakeGpio::new(Rc::clone(&clock));
        for line in [CLK, DT] {
            g.configure(line, INPUT).unwrap();
            g.lines.get_mut(&line).unwrap().level = true;
        }
        let pins = Pins { sw: None, ..PINS };
        let mut enc = Encoder::new(g, pins, Steps::Four, 9, 1);
        enc.init().unwrap();
        let deadline = clock.now().checked_add(Duration::from_millis(5)).unwrap();
        assert_eq!(enc.next_event(&*clock, deadline), Ok(None));
        assert_eq!(clock.now(), deadline);
    }

    #[test]
    fn lost_edges_resynchronise_and_mark_the_next_event() {
        let mut r = Rig::new(Steps::Four);
        r.turn(1);
        // The transport lost edges somewhere in a turn that left the shaft
        // in the next detent; the levels say rest.
        r.enc.gpio.outcomes.push_back(Some(BusError::Overflow));
        let first = r.events();
        assert_eq!(deltas(&first), [1]);
        assert!(first[0].flags().contains(Flags::SEQ_GAP));
        r.turn(1);
        let next = r.events();
        assert_eq!(deltas(&next), [1]);
        assert!(!next[0].flags().contains(Flags::SEQ_GAP));
    }

    #[test]
    fn a_press_lost_with_the_edges_is_found_on_resync() {
        let mut r = Rig::new(Steps::Four);
        r.enc.gpio.lines.get_mut(&SW).unwrap().level = false;
        r.enc.gpio.outcomes.push_back(Some(BusError::Overflow));
        let ev = r.events();
        assert_eq!(ev.len(), 1);
        assert_eq!((ev[0].event_type(), ev[0].value), (Some(EventType::State), 1));
        assert!(ev[0].flags().contains(Flags::SEQ_GAP));
    }

    #[test]
    fn a_two_contact_difference_on_resync_is_counted() {
        let mut r = Rig::new(Steps::Four);
        // Both contacts went low while the edges were lost.
        for line in [CLK, DT] {
            r.enc.gpio.lines.get_mut(&line).unwrap().level = false;
        }
        r.enc.gpio.outcomes.push_back(Some(BusError::Overflow));
        r.events();
        assert_eq!(r.enc.rejected(), 1);
    }

    #[test]
    fn other_bus_errors_are_returned() {
        let mut r = Rig::new(Steps::Four);
        r.enc.gpio.outcomes.push_back(Some(BusError::Io));
        let deadline = r.clock.now().checked_add(Duration::from_secs(1)).unwrap();
        assert_eq!(
            r.enc.next_event(&*r.clock, deadline),
            Err(Error::Bus(BusError::Io))
        );
    }
}
