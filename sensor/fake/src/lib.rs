// SPDX-License-Identifier: BSD-3-Clause
//! A fake clock and fake buses for driver tests.
//!
//! The I2C bus routes each transfer to a [`FakeDevice`] by address.
//! [`RegisterDevice`] models the common register-pointer chip; a per-device
//! hook sees every register write, so a test can model what the chip does in
//! response (start a conversion, reset). Chips that behave differently
//! implement [`FakeDevice`] themselves.
//!
//! [`FakeGpio`] shares the clock with the test: edges that a modelled device
//! produces are scheduled on it and delivered when their time comes.

#![forbid(unsafe_code)]

use std::cell::Cell;
use std::collections::BTreeMap;
use std::rc::Rc;
use std::time::Duration;

use sensor_core::bus::{
    BusError, Clock, Direction, Gpio, GpioEvent, I2c, Instant, LineConfig, TsSource,
};

/// A fake bus panics after this many transfers, so a driver that ignores its
/// deadline fails its test instead of filling memory.
pub const RUNAWAY: usize = 100_000;

/// Time moves only when a driver sleeps.
#[derive(Debug, Default)]
pub struct FakeClock {
    now: Cell<i64>,
}

impl FakeClock {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn advance(&self, d: Duration) {
        self.now.set(self.now.get() + d.as_nanos() as i64);
    }
}

impl Clock for FakeClock {
    fn now(&self) -> Instant {
        Instant::from_nanos(self.now.get())
    }

    fn sleep_until(&self, t: Instant) {
        if t.as_nanos() > self.now.get() {
            self.now.set(t.as_nanos());
        }
    }
}

/// A device on the fake bus. It sees each transfer addressed to it.
pub trait FakeDevice {
    fn addr(&self) -> u8;
    fn transfer(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), BusError>;
}

/// Called after each register write with the register, the value and the
/// device's register file.
pub type WriteHook = Box<dyn FnMut(u8, u8, &mut [u8; 256])>;

/// A chip with a register pointer: a write sets the pointer from its first
/// byte and stores the rest, a read returns bytes from the pointer, both
/// auto-incrementing.
pub struct RegisterDevice {
    pub addr: u8,
    pub regs: [u8; 256],
    ptr: u8,
    hook: Option<WriteHook>,
}

impl RegisterDevice {
    pub fn new(addr: u8) -> Self {
        RegisterDevice {
            addr,
            regs: [0; 256],
            ptr: 0,
            hook: None,
        }
    }

    pub fn with_hook(mut self, hook: WriteHook) -> Self {
        self.hook = Some(hook);
        self
    }
}

impl FakeDevice for RegisterDevice {
    fn addr(&self) -> u8 {
        self.addr
    }

    fn transfer(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), BusError> {
        if let Some((&reg, data)) = write.split_first() {
            self.ptr = reg;
            for &v in data {
                let r = self.ptr;
                self.regs[usize::from(r)] = v;
                if let Some(hook) = self.hook.as_mut() {
                    hook(r, v, &mut self.regs);
                }
                self.ptr = self.ptr.wrapping_add(1);
            }
        }
        for b in read.iter_mut() {
            *b = self.regs[usize::from(self.ptr)];
            self.ptr = self.ptr.wrapping_add(1);
        }
        Ok(())
    }
}

/// One completed transfer, for assertions.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Transfer {
    pub addr: u8,
    pub write: Vec<u8>,
    pub read_len: usize,
}

#[derive(Default)]
pub struct FakeI2c {
    pub devices: Vec<Box<dyn FakeDevice>>,
    pub log: Vec<Transfer>,
    /// Returned by the next transfer instead of performing it.
    pub fail_next: Option<BusError>,
}

impl FakeI2c {
    pub fn new(devices: Vec<Box<dyn FakeDevice>>) -> Self {
        FakeI2c {
            devices,
            ..Self::default()
        }
    }

    pub fn with_device(dev: impl FakeDevice + 'static) -> Self {
        Self::new(vec![Box::new(dev)])
    }

    /// Register writes seen so far, as (register, value) pairs. Meaningful
    /// for register-pointer devices only.
    pub fn register_writes(&self, addr: u8) -> Vec<(u8, u8)> {
        self.log
            .iter()
            .filter(|t| t.addr == addr && t.write.len() > 1)
            .flat_map(|t| {
                t.write[1..]
                    .iter()
                    .enumerate()
                    .map(move |(i, v)| (t.write[0] + i as u8, *v))
            })
            .collect()
    }
}

impl I2c for FakeI2c {
    fn transfer(
        &mut self,
        addr: u8,
        write: &[u8],
        read: &mut [u8],
        _deadline: Instant,
    ) -> Result<(), BusError> {
        if addr > 0x7f || (write.is_empty() && read.is_empty()) {
            return Err(BusError::Invalid);
        }
        if let Some(e) = self.fail_next.take() {
            return Err(e);
        }
        assert!(
            self.log.len() < RUNAWAY,
            "runaway: {RUNAWAY} transfers without stopping"
        );
        let dev = self
            .devices
            .iter_mut()
            .find(|d| d.addr() == addr)
            .ok_or(BusError::Nak)?;
        dev.transfer(write, read)?;
        self.log.push(Transfer {
            addr,
            write: write.to_vec(),
            read_len: read.len(),
        });
        Ok(())
    }
}

/// Answers one 1-Wire transaction: the bytes written after the reset, and a
/// buffer for the bytes to read back.
pub type OneWireResponder = Box<dyn FnMut(&[u8], &mut [u8]) -> Result<(), BusError>>;

/// A 1-Wire bus with at most one device, modelled by a responder. With no
/// responder the reset sees no presence pulse.
#[derive(Default)]
pub struct FakeOneWire {
    pub device: Option<OneWireResponder>,
    /// Bytes written by each transaction, and how many were read back.
    pub log: Vec<(Vec<u8>, usize)>,
}

impl FakeOneWire {
    pub fn with_device(responder: OneWireResponder) -> Self {
        FakeOneWire {
            device: Some(responder),
            log: Vec::new(),
        }
    }
}

impl sensor_core::bus::OneWire for FakeOneWire {
    fn transaction(
        &mut self,
        write: &[u8],
        read: &mut [u8],
        _deadline: Instant,
    ) -> Result<(), BusError> {
        assert!(
            self.log.len() < RUNAWAY,
            "runaway: {RUNAWAY} transactions without stopping"
        );
        let dev = self.device.as_mut().ok_or(BusError::NoPresence)?;
        dev(write, read)?;
        self.log.push((write.to_vec(), read.len()));
        Ok(())
    }
}

/// Called when a driver sets an output line, with the line, its new level and
/// the time. Returns the edges the modelled device produces in response, as
/// (line, level after the edge, delay from now).
pub type GpioResponder = Box<dyn FnMut(u32, bool, Instant) -> Vec<(u32, bool, Duration)>>;

/// One line of a [`FakeGpio`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FakeLine {
    pub config: LineConfig,
    pub level: bool,
}

/// GPIO lines on one fake controller. A blocking wait is modelled by moving
/// the shared clock forward to the next scheduled edge, or to the deadline.
pub struct FakeGpio {
    clock: Rc<FakeClock>,
    pub lines: BTreeMap<u32, FakeLine>,
    /// Edges not yet delivered: (time in ns, line, level after the edge).
    scheduled: Vec<(i64, u32, bool)>,
    pub responder: Option<GpioResponder>,
    /// Every `set`, as (line, level, time in ns).
    pub log: Vec<(u32, bool, i64)>,
    /// Returned by the next `wait_edges` instead of waiting.
    pub fail_next: Option<BusError>,
    waits: usize,
}

impl FakeGpio {
    pub fn new(clock: Rc<FakeClock>) -> Self {
        FakeGpio {
            clock,
            lines: BTreeMap::new(),
            scheduled: Vec::new(),
            responder: None,
            log: Vec::new(),
            fail_next: None,
            waits: 0,
        }
    }

    pub fn with_responder(clock: Rc<FakeClock>, responder: GpioResponder) -> Self {
        FakeGpio {
            responder: Some(responder),
            ..Self::new(clock)
        }
    }

    /// Schedules an edge `after` the current time, as an outside signal would.
    pub fn schedule(&mut self, line: u32, level: bool, after: Duration) {
        let at = self.clock.now().as_nanos() + after.as_nanos() as i64;
        self.scheduled.push((at, line, level));
        self.scheduled.sort_by_key(|e| e.0);
    }

    /// Removes the edges that are due and applies their levels. Returns those
    /// that change a line's level and that the line is configured to report.
    fn take_due(&mut self, out: &mut [GpioEvent]) -> usize {
        let now = self.clock.now().as_nanos();
        let mut n = 0;
        while n < out.len() && self.scheduled.first().is_some_and(|e| e.0 <= now) {
            let (at, line, level) = self.scheduled.remove(0);
            let Some(l) = self.lines.get_mut(&line) else {
                continue;
            };
            if l.level == level {
                continue;
            }
            l.level = level;
            let c = l.config;
            let wanted = c.direction == Direction::Input
                && ((level && c.edges.rising) || (!level && c.edges.falling));
            if wanted {
                out[n] = GpioEvent {
                    timestamp: at,
                    line,
                    level: u8::from(level),
                    ts_source: TsSource::Interrupt as u8,
                    reserved: 0,
                };
                n += 1;
            }
        }
        n
    }
}

impl Gpio for FakeGpio {
    fn configure(&mut self, line: u32, config: LineConfig) -> Result<(), BusError> {
        let level = self.lines.get(&line).is_some_and(|l| l.level);
        self.lines.insert(line, FakeLine { config, level });
        Ok(())
    }

    fn set(&mut self, line: u32, high: bool) -> Result<(), BusError> {
        let l = self.lines.get_mut(&line).ok_or(BusError::Invalid)?;
        if l.config.direction != Direction::Output {
            return Err(BusError::Invalid);
        }
        l.level = high;
        let now = self.clock.now();
        self.log.push((line, high, now.as_nanos()));
        if let Some(r) = self.responder.as_mut() {
            for (line, level, after) in r(line, high, now) {
                self.scheduled
                    .push((now.as_nanos() + after.as_nanos() as i64, line, level));
            }
            self.scheduled.sort_by_key(|e| e.0);
        }
        Ok(())
    }

    fn get(&mut self, line: u32) -> Result<bool, BusError> {
        if !self.lines.contains_key(&line) {
            return Err(BusError::Invalid);
        }
        // Apply due edges without consuming the ones a wait should report.
        let now = self.clock.now().as_nanos();
        let mut level = self.lines[&line].level;
        for &(at, l, v) in &self.scheduled {
            if at <= now && l == line {
                level = v;
            }
        }
        Ok(level)
    }

    fn wait_edges(&mut self, out: &mut [GpioEvent], deadline: Instant) -> Result<usize, BusError> {
        if out.is_empty() {
            return Err(BusError::Invalid);
        }
        if let Some(e) = self.fail_next.take() {
            return Err(e);
        }
        self.waits += 1;
        assert!(
            self.waits < RUNAWAY,
            "runaway: {RUNAWAY} waits without stopping"
        );
        loop {
            let n = self.take_due(out);
            if n > 0 {
                return Ok(n);
            }
            match self.scheduled.first() {
                Some(&(at, _, _)) if at <= deadline.as_nanos() => {
                    self.clock.sleep_until(Instant::from_nanos(at));
                }
                _ => {
                    self.clock.sleep_until(deadline);
                    return Err(BusError::Timeout);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sensor_core::bus::{Bias, Edges};

    const IN_BOTH: LineConfig = LineConfig {
        direction: Direction::Input,
        bias: Bias::None,
        edges: Edges::BOTH,
    };
    const OUT: LineConfig = LineConfig {
        direction: Direction::Output,
        bias: Bias::None,
        edges: Edges::NONE,
    };

    fn at(clock: &FakeClock, d: Duration) -> Instant {
        clock.now().checked_add(d).unwrap()
    }

    #[test]
    fn a_wait_moves_the_clock_to_the_next_edge() {
        let clock = Rc::new(FakeClock::new());
        let mut g = FakeGpio::new(Rc::clone(&clock));
        g.configure(4, IN_BOTH).unwrap();
        g.schedule(4, true, Duration::from_micros(300));
        let mut ev = [GpioEvent::default(); 4];
        let n = g
            .wait_edges(&mut ev, at(&clock, Duration::from_millis(1)))
            .unwrap();
        assert_eq!(n, 1);
        assert_eq!((ev[0].line, ev[0].level, ev[0].timestamp), (4, 1, 300_000));
        assert_eq!(clock.now().as_nanos(), 300_000);
        assert!(g.get(4).unwrap());
    }

    #[test]
    fn a_wait_past_the_last_edge_times_out_at_the_deadline() {
        let clock = Rc::new(FakeClock::new());
        let mut g = FakeGpio::new(Rc::clone(&clock));
        g.configure(4, IN_BOTH).unwrap();
        g.schedule(4, true, Duration::from_millis(5));
        let mut ev = [GpioEvent::default(); 4];
        let deadline = at(&clock, Duration::from_millis(1));
        assert_eq!(g.wait_edges(&mut ev, deadline), Err(BusError::Timeout));
        assert_eq!(clock.now(), deadline);
    }

    #[test]
    fn unwanted_and_non_changing_edges_are_not_reported() {
        let clock = Rc::new(FakeClock::new());
        let mut g = FakeGpio::new(Rc::clone(&clock));
        let rising_only = LineConfig {
            edges: Edges::RISING,
            ..IN_BOTH
        };
        g.configure(4, rising_only).unwrap();
        g.schedule(4, false, Duration::from_micros(1)); // no change
        g.schedule(4, true, Duration::from_micros(2));
        g.schedule(4, false, Duration::from_micros(3)); // falling, not wanted
        g.schedule(4, true, Duration::from_micros(4));
        let mut ev = [GpioEvent::default(); 4];
        let deadline = at(&clock, Duration::from_millis(1));
        let mut seen = Vec::new();
        while let Ok(n) = g.wait_edges(&mut ev, deadline) {
            seen.extend(ev[..n].iter().map(|e| e.timestamp));
        }
        assert_eq!(seen, [2_000, 4_000]);
    }

    #[test]
    fn a_responder_answers_an_output() {
        let clock = Rc::new(FakeClock::new());
        let mut g = FakeGpio::with_responder(
            Rc::clone(&clock),
            Box::new(|line, high, _| {
                if line == 17 && !high {
                    vec![(4, true, Duration::from_micros(10))]
                } else {
                    vec![]
                }
            }),
        );
        g.configure(17, OUT).unwrap();
        g.configure(4, IN_BOTH).unwrap();
        assert_eq!(g.set(4, true), Err(BusError::Invalid), "input line");
        g.set(17, true).unwrap();
        g.set(17, false).unwrap();
        let mut ev = [GpioEvent::default(); 1];
        assert_eq!(
            g.wait_edges(&mut ev, at(&clock, Duration::from_millis(1))),
            Ok(1)
        );
        assert_eq!(ev[0].timestamp, 10_000);
    }
}
