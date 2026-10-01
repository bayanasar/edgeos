// SPDX-License-Identifier: BSD-3-Clause
//! HC-SR04 ultrasonic ranging module, on two GPIO lines.
//!
//! Follows the HC-SR04 user's manual: a pulse of at least 10 us on TRIG
//! starts a measurement, the module sends a 40 kHz burst, and ECHO stays high
//! for the time the sound takes to return. Distance is half that time
//! multiplied by the speed of sound. The manual gives a range of 2 cm to
//! 4 m and suggests at least 60 ms between measurements, so that a late
//! echo is not taken for the next one; the driver enforces that interval.
//!
//! The pulse width is measured from the two edge timestamps, not by polling,
//! so its accuracy is that of the transport's timestamps. ECHO is a 5 V
//! output on the common module and must be divided down for a 3.3 V input.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::time::Duration;

use sensor_core::bus::{
    Bias, BusError, Clock, Direction, Edges, Gpio, GpioEvent, Instant, LineConfig,
};
use sensor_core::sample::{Channel, Flags, Kind, MAX_CHANNELS, Sample, Unit};

const TRIGGER_PULSE: Duration = Duration::from_micros(10);
const CYCLE: Duration = Duration::from_millis(60);
/// From the end of the trigger pulse to the echo's rising edge. The burst
/// itself is 8 cycles at 40 kHz, 200 us; the margin covers module variants.
const ECHO_START_LIMIT: Duration = Duration::from_millis(10);
/// Longest echo accepted. 4 m and back is about 23 ms; modules that hear
/// nothing commonly drop ECHO after about 38 ms.
const ECHO_LIMIT: Duration = Duration::from_millis(50);
const RANGE_MIN_MM: i64 = 20;
const RANGE_MAX_MM: i64 = 4_000;
/// Stale edges read and discarded before a trigger, at most.
const DRAIN_LIMIT: usize = 16;

/// Speed of sound in dry air at 20 degC, from c = 331.3 + 0.606 T m/s.
pub const SPEED_OF_SOUND_20C_MM_S: i64 = 343_420;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Bus(BusError),
    /// ECHO did not rise after the trigger.
    NoEcho,
    /// ECHO rose but did not fall within the longest accepted echo.
    EchoStuck,
    /// Edges kept arriving while the driver waited for the line to settle.
    Noisy,
    /// The clock cannot represent the deadline.
    Clock,
}

impl From<BusError> for Error {
    fn from(e: BusError) -> Self {
        Error::Bus(e)
    }
}

pub struct HcSr04<G> {
    gpio: G,
    trig: u32,
    echo: u32,
    sensor_id: u32,
    epoch: u32,
    seq: u32,
    speed_mm_s: i64,
    last_trigger: Option<Instant>,
    initialized: bool,
    announce_epoch: bool,
}

impl<G: Gpio> HcSr04<G> {
    /// No line is touched until [`HcSr04::init`].
    pub fn new(gpio: G, trig: u32, echo: u32, sensor_id: u32, epoch: u32) -> Self {
        HcSr04 {
            gpio,
            trig,
            echo,
            sensor_id,
            epoch,
            seq: 0,
            speed_mm_s: SPEED_OF_SOUND_20C_MM_S,
            last_trigger: None,
            initialized: false,
            announce_epoch: true,
        }
    }

    pub fn release(self) -> G {
        self.gpio
    }

    /// Sets the speed of sound from the air temperature in 0.01 degC, as a
    /// thermometer reports it: 331.3 + 0.606 T m/s.
    pub fn set_air_temperature(&mut self, centi_c: i32) {
        self.speed_mm_s = 331_300 + 606 * i64::from(centi_c) / 100;
    }

    pub fn speed_of_sound_mm_s(&self) -> i64 {
        self.speed_mm_s
    }

    /// TRIG as an output driven low, ECHO as an input reporting both edges.
    pub fn init(&mut self) -> Result<(), Error> {
        self.gpio.configure(
            self.trig,
            LineConfig {
                direction: Direction::Output,
                bias: Bias::None,
                edges: Edges::NONE,
            },
        )?;
        self.gpio.set(self.trig, false)?;
        self.gpio.configure(
            self.echo,
            LineConfig {
                direction: Direction::Input,
                bias: Bias::None,
                edges: Edges::BOTH,
            },
        )?;
        self.initialized = true;
        Ok(())
    }

    /// One measurement. Channel 0 is the distance in metres with three
    /// decimals (millimetres); its raw value is the echo width in ns. The
    /// timestamp is the echo's rising edge.
    pub fn read<C: Clock>(&mut self, clock: &C) -> Result<Sample, Error> {
        if !self.initialized {
            self.init()?;
        }
        if let Some(t) = self.last_trigger {
            clock.sleep_until(t.checked_add(CYCLE).ok_or(Error::Clock)?);
        }
        self.drain(clock)?;

        self.gpio.set(self.trig, true)?;
        clock.sleep_until(after(clock, TRIGGER_PULSE)?);
        self.gpio.set(self.trig, false)?;
        self.last_trigger = Some(clock.now());

        let (rise, fall) = self.echo_edges(clock)?;
        let width_ns = fall.timestamp - rise.timestamp;
        let mm = width_ns * self.speed_mm_s / 2 / 1_000_000_000;

        let mut flags = if self.announce_epoch {
            Flags::NEW_EPOCH
        } else {
            Flags::default()
        };
        if !(RANGE_MIN_MM..=RANGE_MAX_MM).contains(&mm) {
            flags = flags.union(Flags::SATURATED);
        }
        let mut ch = [Channel::default(); MAX_CHANNELS];
        ch[0] = Channel {
            // Both fit: the echo is at most ECHO_LIMIT, 5e7 ns and about 8.6 m.
            raw: width_ns as i32,
            value: mm as i32,
            unit: Unit::Metre as u16,
            exp: -3,
            reserved: 0,
        };
        let sample = Sample {
            timestamp: rise.timestamp,
            sensor_id: self.sensor_id,
            seq: self.seq,
            epoch: self.epoch,
            kind: Kind::Distance as u16,
            flags: flags.0,
            ts_source: rise.ts_source,
            nchannels: 1,
            ch,
            ..Sample::default()
        };
        self.announce_epoch = false;
        self.seq = self.seq.wrapping_add(1);
        Ok(sample)
    }

    /// Discards edges that arrived since the last measurement.
    fn drain<C: Clock>(&mut self, clock: &C) -> Result<(), Error> {
        let mut buf = [GpioEvent::default(); 8];
        for _ in 0..DRAIN_LIMIT {
            match self.gpio.wait_edges(&mut buf, clock.now()) {
                Ok(_) | Err(BusError::Overflow) => {}
                Err(BusError::Timeout) => return Ok(()),
                Err(e) => return Err(e.into()),
            }
        }
        Err(Error::Noisy)
    }

    /// The echo's rising edge and the falling edge after it.
    fn echo_edges<C: Clock>(&mut self, clock: &C) -> Result<(GpioEvent, GpioEvent), Error> {
        let mut deadline = after(clock, ECHO_START_LIMIT)?;
        let mut rise: Option<GpioEvent> = None;
        let mut buf = [GpioEvent::default(); 8];
        loop {
            let n = match self.gpio.wait_edges(&mut buf, deadline) {
                Ok(n) => n,
                Err(BusError::Timeout) if rise.is_none() => return Err(Error::NoEcho),
                Err(BusError::Timeout) => return Err(Error::EchoStuck),
                Err(e) => return Err(e.into()),
            };
            for e in buf[..n].iter().filter(|e| e.line == self.echo) {
                match (rise, e.level) {
                    (None, 1) => {
                        rise = Some(*e);
                        deadline = Instant::from_nanos(e.timestamp)
                            .checked_add(ECHO_LIMIT)
                            .ok_or(Error::Clock)?;
                    }
                    (Some(r), 0) => return Ok((r, *e)),
                    _ => {}
                }
            }
        }
    }
}

fn after<C: Clock>(clock: &C, d: Duration) -> Result<Instant, Error> {
    clock.now().checked_add(d).ok_or(Error::Clock)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sensor_fake::{FakeClock, FakeGpio};
    use std::rc::Rc;

    const TRIG: u32 = 23;
    const ECHO: u32 = 24;
    const BURST: Duration = Duration::from_micros(450);

    /// Width of the echo for a target `mm` away at 20 degC, in ns.
    fn width_for(mm: i64) -> Duration {
        Duration::from_nanos((2 * mm * 1_000_000_000 / SPEED_OF_SOUND_20C_MM_S) as u64)
    }

    /// A module that answers each trigger pulse of at least 10 us with an
    /// echo of the given width; `None` stays silent.
    fn module(clock: &Rc<FakeClock>, width: Option<Duration>) -> FakeGpio {
        let rose = std::cell::Cell::new(None::<Instant>);
        FakeGpio::with_responder(
            Rc::clone(clock),
            Box::new(move |line, high, now| {
                if line != TRIG {
                    return vec![];
                }
                if high {
                    rose.set(Some(now));
                    return vec![];
                }
                let long_enough = rose
                    .take()
                    .is_some_and(|t| now.saturating_duration_since(t) >= TRIGGER_PULSE);
                match width {
                    Some(w) if long_enough => vec![(ECHO, true, BURST), (ECHO, false, BURST + w)],
                    _ => vec![],
                }
            }),
        )
    }

    fn sensor(clock: &Rc<FakeClock>, width: Option<Duration>) -> HcSr04<FakeGpio> {
        HcSr04::new(module(clock, width), TRIG, ECHO, 7, 1)
    }

    #[test]
    fn one_metre_reads_as_1000_mm() {
        let clock = Rc::new(FakeClock::new());
        let mut s = sensor(&clock, Some(width_for(1_000)));
        let sample = s.read(&*clock).unwrap();
        let ch = sample.channels().unwrap();
        assert!((ch[0].value - 1_000).abs() <= 1, "{}", ch[0].value);
        assert_eq!(ch[0].raw as u128, width_for(1_000).as_nanos());
        assert_eq!((ch[0].unit, ch[0].exp), (Unit::Metre as u16, -3));
        assert_eq!(sample.kind(), Some(Kind::Distance));
        assert!(!sample.flags().contains(Flags::SATURATED));
        // Timestamped at the echo's rising edge, after the trigger and burst.
        assert_eq!(sample.timestamp, (TRIGGER_PULSE + BURST).as_nanos() as i64);
    }

    #[test]
    fn the_manuals_formula_agrees() {
        // The manual's rule of thumb: distance in cm = width in us / 58.
        let clock = Rc::new(FakeClock::new());
        let mut s = sensor(&clock, Some(Duration::from_micros(5_800)));
        let mm = s.read(&*clock).unwrap().channels().unwrap()[0].value;
        assert!((mm - 1_000).abs() <= 5, "{mm}");
    }

    #[test]
    fn the_trigger_pulse_is_at_least_10_us() {
        let clock = Rc::new(FakeClock::new());
        let mut s = sensor(&clock, Some(width_for(500)));
        s.read(&*clock).unwrap();
        let log = &s.release().log;
        let pulses: Vec<_> = log.iter().filter(|e| e.0 == TRIG).collect();
        // init drives TRIG low, then one high-low pulse.
        assert_eq!(
            pulses.iter().map(|e| e.1).collect::<Vec<_>>(),
            [false, true, false]
        );
        assert!(pulses[2].2 - pulses[1].2 >= 10_000);
    }

    #[test]
    fn measurements_are_at_least_60_ms_apart() {
        let clock = Rc::new(FakeClock::new());
        let mut s = sensor(&clock, Some(width_for(300)));
        s.read(&*clock).unwrap();
        s.read(&*clock).unwrap();
        let log = &s.release().log;
        let falls: Vec<i64> = log
            .iter()
            .filter(|e| e.0 == TRIG && !e.1)
            .skip(1)
            .map(|e| e.2)
            .collect();
        assert!(falls[1] - falls[0] >= 60_000_000, "{falls:?}");
    }

    #[test]
    fn silence_is_no_echo() {
        let clock = Rc::new(FakeClock::new());
        let mut s = sensor(&clock, None);
        assert_eq!(s.read(&*clock), Err(Error::NoEcho));
    }

    #[test]
    fn an_echo_that_never_falls_is_stuck() {
        let clock = Rc::new(FakeClock::new());
        let mut s = sensor(&clock, Some(Duration::from_millis(80)));
        assert_eq!(s.read(&*clock), Err(Error::EchoStuck));
    }

    #[test]
    fn out_of_range_is_saturated() {
        for mm in [10, 4_500] {
            let clock = Rc::new(FakeClock::new());
            let mut s = sensor(&clock, Some(width_for(mm)));
            let sample = s.read(&*clock).unwrap();
            assert!(sample.flags().contains(Flags::SATURATED), "{mm} mm");
        }
    }

    #[test]
    fn stale_edges_before_the_trigger_are_ignored() {
        let clock = Rc::new(FakeClock::new());
        let mut s = sensor(&clock, Some(width_for(1_000)));
        s.init().unwrap();
        // A glitch on ECHO before the measurement.
        s.gpio.schedule(ECHO, true, Duration::ZERO);
        s.gpio.schedule(ECHO, false, Duration::from_micros(3));
        clock.advance(Duration::from_millis(1));
        let mm = s.read(&*clock).unwrap().channels().unwrap()[0].value;
        assert!((mm - 1_000).abs() <= 1, "{mm}");
    }

    #[test]
    fn colder_air_shortens_the_distance() {
        let clock = Rc::new(FakeClock::new());
        let mut s = sensor(&clock, Some(width_for(1_000)));
        s.set_air_temperature(2_000);
        assert_eq!(s.speed_of_sound_mm_s(), SPEED_OF_SOUND_20C_MM_S);
        s.set_air_temperature(-1_050);
        assert_eq!(s.speed_of_sound_mm_s(), 324_937); // 331.3 - 0.606 * 10.5
        s.set_air_temperature(0);
        assert_eq!(s.speed_of_sound_mm_s(), 331_300);
        let mm = s.read(&*clock).unwrap().channels().unwrap()[0].value;
        // 1000 mm * 331.3 / 343.42
        assert!((mm - 965).abs() <= 1, "{mm}");
    }

    #[test]
    fn lost_edges_are_tolerated_while_draining_but_not_while_measuring() {
        let clock = Rc::new(FakeClock::new());
        let mut s = sensor(&clock, Some(width_for(1_000)));
        s.gpio.outcomes.push_back(Some(BusError::Overflow));
        assert!(s.read(&*clock).is_ok());
        // The drain runs normally, the first wait for the echo loses edges.
        s.gpio.outcomes.extend([None, Some(BusError::Overflow)]);
        assert_eq!(s.read(&*clock), Err(Error::Bus(BusError::Overflow)));
    }

    #[test]
    fn a_line_that_never_settles_is_noisy() {
        let clock = Rc::new(FakeClock::new());
        let mut s = sensor(&clock, Some(width_for(1_000)));
        s.gpio
            .outcomes
            .extend([Some(BusError::Overflow); DRAIN_LIMIT]);
        assert_eq!(s.read(&*clock), Err(Error::Noisy));
    }

    #[test]
    fn samples_carry_sequence_and_epoch() {
        let clock = Rc::new(FakeClock::new());
        let mut s = sensor(&clock, Some(width_for(800)));
        let a = s.read(&*clock).unwrap();
        let b = s.read(&*clock).unwrap();
        assert_eq!((a.seq, b.seq, a.epoch), (0, 1, 1));
        assert!(a.flags().contains(Flags::NEW_EPOCH));
        assert!(!b.flags().contains(Flags::NEW_EPOCH));
    }
}
