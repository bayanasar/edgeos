// SPDX-License-Identifier: BSD-3-Clause
//! DHT11 and DHT22 (AM2302) temperature and humidity sensors, on one GPIO
//! line.
//!
//! Timings and formats are from two Aosong manuals: "Temperature and
//! Humidity Module DHT11 Product Manual" V1.3 (2017-03-31), whose table 10
//! gives minimum and maximum times, and "Digital relative humidity &
//! temperature sensor AM2302/DHT22", which gives typical times only. The two
//! parts cannot be told apart from their answer, so the caller names the
//! model.
//!
//! # Protocol
//!
//! The data line idles high through a pull-up. The host drives it low
//! (DHT11: at least 18 ms and at most 30 ms, 20 typical; AM2302: "at least
//! 1~10 ms") and releases it. The sensor answers 10 to 40 us later by holding
//! the line low for about 80 us and high for about 80 us, then sends 40 bits,
//! most significant first. Each bit is a low of about 50 us followed by a high
//! whose length is the bit: about 24 us (DHT11: 23 to 27) for 0, about 70 us
//! (DHT11: 68 to 74) for 1. A last low of about 50 us ends the frame. Both
//! manuals ask for more than 2 s between reads, and 1 s after power-up before
//! the first.
//!
//! # Decoding
//!
//! The driver watches falling edges only and reads each bit from the time
//! between consecutive falls: a whole bit period, 73 to 85 us for 0 and 118
//! to 132 us for 1 by the DHT11 table, split at 100 us. A falling edge needs
//! no level read and is at least 73 us from the next, where a high pulse can
//! be as short as 23 us. On Linux this matters: for an edge request on both
//! edges the kernel reads the level in its interrupt thread and keeps the
//! line's interrupt masked until that thread finishes, so a short pulse is
//! only as good as the thread's latency.
//!
//! A frame is the response's falling edge, the fall that ends the response,
//! and one fall at the end of each bit: 42 edges. The first may be lost,
//! because the transport arms edge detection only after the release and the
//! sensor answers within tens of microseconds, so 41 edges are also
//! accepted; with 42 the response's length is checked too. Any other count is
//! rejected, as is a period outside the accepted window or a failed checksum.
//! The windows are the DHT11 table's limits widened by [`MARGIN`] for
//! timestamp jitter; the AM2302's typical times fall inside them. The margin
//! is a choice, not a data-sheet value.
//!
//! # Values
//!
//! DHT11: humidity in whole percent (the decimal byte is 0 by the manual),
//! temperature as an integer byte and a decimal byte whose top bit marks a
//! negative value. The manual does not give the decimal's scale; the driver
//! reads it as tenths and rejects a value above 9. The manual says each read
//! returns the previous measurement, so the first sample after
//! [`Dht::init`] is marked [`Flags::STALE`]: its age is unknown.
//!
//! DHT22: 16-bit humidity and temperature in tenths, the temperature's top
//! bit marking a negative value. Its manual does not say which measurement a
//! read returns, so nothing is marked.
//!
//! A frame that passes the checksum but holds a value the model cannot
//! produce, such as humidity above 100 percent, is rejected: it usually
//! means the wrong model was named. Values outside the model's measuring
//! range are kept and marked [`Flags::SATURATED`].

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::time::Duration;

use sensor_core::bus::{
    Bias, BusError, Clock, Direction, Edges, Gpio, GpioEvent, Instant, LineConfig,
};
use sensor_core::sample::{Channel, Flags, Kind, MAX_CHANNELS, Sample, Unit};

/// Between the starts of two reads; both manuals ask for more than 2 s.
pub const INTERVAL: Duration = Duration::from_secs(2);
/// How long the driver listens after releasing the line. A frame is at most
/// 35 + 88 + 92 + 40 * (58 + 74) + 56 us, about 5.6 ms, by the DHT11 table.
const FRAME_WINDOW: Duration = Duration::from_millis(10);
/// Widening of every accepted window, for timestamp jitter. Below 13 us, so
/// that the response window and the bit window do not meet.
pub const MARGIN: Duration = Duration::from_micros(12);

const US: i64 = 1_000;
const MARGIN_NS: i64 = MARGIN.as_nanos() as i64;
/// DHT11 table 10: shortest 0 bit (low 50 + high 23 us) and longest 1 bit
/// (low 58 + high 74 us).
const BIT_MIN_NS: i64 = 73 * US - MARGIN_NS;
const BIT_MAX_NS: i64 = 132 * US + MARGIN_NS;
/// Between the longest 0 (58 + 27 = 85 us) and the shortest 1 (50 + 68 =
/// 118 us).
const ONE_FROM_NS: i64 = 100 * US;
/// DHT11 table 10: response low 78 to 88 us plus high 80 to 92 us. Its
/// minimum stays above the longest accepted bit, so a frame that starts one
/// edge late fails on its first period.
const RESPONSE_MIN_NS: i64 = 158 * US - MARGIN_NS;
const RESPONSE_MAX_NS: i64 = 180 * US + MARGIN_NS;
const _: () = assert!(RESPONSE_MIN_NS > BIT_MAX_NS);

const BITS: usize = 40;
/// Falling edges in a whole frame: response, end of response, one per bit.
const FRAME_EDGES: usize = BITS + 2;
/// Stale edges read and discarded before a start signal, at most.
const DRAIN_LIMIT: usize = 16;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Model {
    Dht11,
    /// DHT22, also sold as AM2302.
    Dht22,
}

impl Model {
    /// How long the host holds the line low. DHT11: 20 ms, the table's
    /// typical value, inside the 18 to 30 ms of its text. AM2302: 10 ms,
    /// which satisfies every reading of "at least 1~10 ms".
    pub const fn start_signal(self) -> Duration {
        match self {
            Model::Dht11 => Duration::from_millis(20),
            Model::Dht22 => Duration::from_millis(10),
        }
    }

    /// Measuring range in tenths: humidity (%RH) and temperature (degC).
    /// DHT11 V1.3: 5 to 95 %RH, -20 to 60 degC. AM2302: 0 to 100 %RH,
    /// -40 to 80 degC.
    const fn range(self) -> ((i32, i32), (i32, i32)) {
        match self {
            Model::Dht11 => ((50, 950), (-200, 600)),
            Model::Dht22 => ((0, 1000), (-400, 800)),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Bus(BusError),
    /// No edge after the start signal: no sensor, or no answer.
    NoResponse,
    /// A frame had this many falling edges instead of 41 or 42.
    EdgeCount(u16),
    /// A period outside its window: bit 0 to 39, or 40 for the response.
    Timing { bit: u8 },
    Checksum,
    /// The checksum passed but the value is impossible for the model.
    Implausible,
    /// Edges kept arriving while the driver waited for the line to settle.
    Noisy,
    /// The clock cannot represent a deadline.
    Clock,
}

impl From<BusError> for Error {
    fn from(e: BusError) -> Self {
        Error::Bus(e)
    }
}

/// What the last read saw, for measuring a transport.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FrameInfo {
    /// Falling edges after the release.
    pub edges: u16,
    /// From the release to the first edge, in ns; 0 without edges.
    pub first_edge_ns: i64,
    /// Shortest and longest period read as 0, and as 1, in ns; zero if none.
    pub zero_ns: (i64, i64),
    pub one_ns: (i64, i64),
}

/// The five bytes of a frame from the falling-edge times, in ns.
fn decode(times: &[i64], info: &mut FrameInfo) -> Result<[u8; 5], Error> {
    let data = match times.len() {
        FRAME_EDGES => {
            let response = times[1] - times[0];
            if !(RESPONSE_MIN_NS..=RESPONSE_MAX_NS).contains(&response) {
                return Err(Error::Timing { bit: BITS as u8 });
            }
            &times[1..]
        }
        n if n == FRAME_EDGES - 1 => times,
        n => return Err(Error::EdgeCount(n.min(usize::from(u16::MAX)) as u16)),
    };
    let mut bytes = [0u8; 5];
    for (i, w) in data.windows(2).enumerate() {
        let period = w[1] - w[0];
        if !(BIT_MIN_NS..=BIT_MAX_NS).contains(&period) {
            return Err(Error::Timing { bit: i as u8 });
        }
        let one = period >= ONE_FROM_NS;
        let range = if one {
            &mut info.one_ns
        } else {
            &mut info.zero_ns
        };
        *range = if range.1 == 0 {
            (period, period)
        } else {
            (range.0.min(period), range.1.max(period))
        };
        bytes[i / 8] = (bytes[i / 8] << 1) | u8::from(one);
    }
    let sum = bytes[..4].iter().fold(0u8, |s, &b| s.wrapping_add(b));
    if sum != bytes[4] {
        return Err(Error::Checksum);
    }
    Ok(bytes)
}

/// Humidity and temperature in tenths, or `None` if the model cannot
/// produce these bytes.
fn convert(model: Model, b: [u8; 5]) -> Option<(i32, i32)> {
    match model {
        Model::Dht11 => {
            let frac = b[3] & 0x7f;
            if b[1] != 0 || frac > 9 || b[0] > 100 {
                return None;
            }
            let t = i32::from(b[2]) * 10 + i32::from(frac);
            let t = if b[3] & 0x80 != 0 { -t } else { t };
            Some((i32::from(b[0]) * 10, t))
        }
        Model::Dht22 => {
            let rh = i32::from(u16::from_be_bytes([b[0], b[1]]));
            if rh > 1000 {
                return None;
            }
            let t = i32::from(u16::from_be_bytes([b[2] & 0x7f, b[3]]));
            let t = if b[2] & 0x80 != 0 { -t } else { t };
            Some((rh, t))
        }
    }
}

const IDLE: LineConfig = LineConfig {
    direction: Direction::Input,
    bias: Bias::PullUp,
    edges: Edges::FALLING,
};

const DRIVE: LineConfig = LineConfig {
    direction: Direction::Output,
    bias: Bias::None,
    edges: Edges::NONE,
};

pub struct Dht<G> {
    gpio: G,
    line: u32,
    model: Model,
    sensor_id: u32,
    epoch: u32,
    seq: u32,
    last_start: Option<Instant>,
    initialized: bool,
    announce_epoch: bool,
    /// The next good sample is the first since init.
    first: bool,
    last_frame: FrameInfo,
}

impl<G: Gpio> Dht<G> {
    /// No line is touched until [`Dht::init`].
    pub fn new(gpio: G, line: u32, model: Model, sensor_id: u32, epoch: u32) -> Self {
        Dht {
            gpio,
            line,
            model,
            sensor_id,
            epoch,
            seq: 0,
            last_start: None,
            initialized: false,
            announce_epoch: true,
            first: true,
            last_frame: FrameInfo::default(),
        }
    }

    pub fn release(self) -> G {
        self.gpio
    }

    pub fn last_frame(&self) -> FrameInfo {
        self.last_frame
    }

    /// The line as an input with a pull-up, reporting falling edges: the
    /// bus's idle state. The module may have its own pull-up as well.
    pub fn init(&mut self) -> Result<(), Error> {
        self.gpio.configure(self.line, IDLE)?;
        self.initialized = true;
        self.first = true;
        Ok(())
    }

    /// One reading. Channel 0 is relative humidity and channel 1 the
    /// temperature, both in tenths; their raw values are the frame's 16-bit
    /// fields. The timestamp is the first edge of the answer. Waits until
    /// [`INTERVAL`] after the previous start signal.
    pub fn read<C: Clock>(&mut self, clock: &C) -> Result<Sample, Error> {
        self.last_frame = FrameInfo::default();
        if !self.initialized {
            self.init()?;
        }
        if let Some(t) = self.last_start {
            clock.sleep_until(t.checked_add(INTERVAL).ok_or(Error::Clock)?);
        }
        self.drain(clock)?;

        self.gpio.configure(self.line, DRIVE)?;
        self.gpio.set(self.line, false)?;
        let low = clock.now();
        self.last_start = Some(low);
        clock.sleep_until(
            low.checked_add(self.model.start_signal())
                .ok_or(Error::Clock)?,
        );
        let release = clock.now();
        self.gpio.configure(self.line, IDLE)?;

        let mut times = [0i64; FRAME_EDGES];
        let mut n = 0usize;
        let mut first = None;
        let deadline = release.checked_add(FRAME_WINDOW).ok_or(Error::Clock)?;
        let mut buf = [GpioEvent::default(); 16];
        loop {
            let got = match self.gpio.wait_edges(&mut buf, deadline) {
                Ok(got) => got,
                Err(BusError::Timeout) => break,
                Err(e) => return Err(e.into()),
            };
            let edges = buf[..got]
                .iter()
                .filter(|e| e.line == self.line && e.timestamp >= release.as_nanos());
            for e in edges {
                if first.is_none() {
                    first = Some(*e);
                }
                if let Some(t) = times.get_mut(n) {
                    *t = e.timestamp;
                }
                n += 1;
            }
        }
        self.last_frame = FrameInfo {
            edges: n.min(usize::from(u16::MAX)) as u16,
            first_edge_ns: first.map_or(0, |e| e.timestamp - release.as_nanos()),
            ..FrameInfo::default()
        };
        let Some(first) = first else {
            return Err(Error::NoResponse);
        };
        if n > FRAME_EDGES {
            return Err(Error::EdgeCount(self.last_frame.edges));
        }
        let bytes = decode(&times[..n], &mut self.last_frame)?;
        let (rh, t) = convert(self.model, bytes).ok_or(Error::Implausible)?;

        let mut flags = if self.announce_epoch {
            Flags::NEW_EPOCH
        } else {
            Flags::default()
        };
        if self.first && self.model == Model::Dht11 {
            flags = flags.union(Flags::STALE);
        }
        let ((rh_min, rh_max), (t_min, t_max)) = self.model.range();
        if !(rh_min..=rh_max).contains(&rh) || !(t_min..=t_max).contains(&t) {
            flags = flags.union(Flags::SATURATED);
        }
        let mut ch = [Channel::default(); MAX_CHANNELS];
        ch[0] = Channel {
            raw: i32::from(u16::from_be_bytes([bytes[0], bytes[1]])),
            value: rh,
            unit: Unit::PercentRh as u16,
            exp: -1,
            reserved: 0,
        };
        ch[1] = Channel {
            raw: i32::from(u16::from_be_bytes([bytes[2], bytes[3]])),
            value: t,
            unit: Unit::DegC as u16,
            exp: -1,
            reserved: 0,
        };
        let sample = Sample {
            timestamp: first.timestamp,
            sensor_id: self.sensor_id,
            seq: self.seq,
            epoch: self.epoch,
            kind: Kind::Humidity as u16,
            flags: flags.0,
            ts_source: first.ts_source,
            nchannels: 2,
            ch,
            ..Sample::default()
        };
        self.announce_epoch = false;
        self.first = false;
        self.seq = self.seq.wrapping_add(1);
        Ok(sample)
    }

    /// Discards edges that arrived since the last read.
    fn drain<C: Clock>(&mut self, clock: &C) -> Result<(), Error> {
        let mut buf = [GpioEvent::default(); 16];
        for _ in 0..DRAIN_LIMIT {
            match self.gpio.wait_edges(&mut buf, clock.now()) {
                Ok(_) | Err(BusError::Overflow) => {}
                Err(BusError::Timeout) => return Ok(()),
                Err(e) => return Err(e.into()),
            }
        }
        Err(Error::Noisy)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sensor_core::bus::TsSource;
    use sensor_fake::{FakeClock, FakeGpio};
    use std::cell::Cell;
    use std::rc::Rc;

    const LINE: u32 = 25;

    /// One sensor's times, in us: release to answer, response low and high,
    /// bit low, 0 high, 1 high, end low.
    #[derive(Clone, Copy)]
    struct Times {
        go: u64,
        rel: u64,
        reh: u64,
        low: u64,
        h0: u64,
        h1: u64,
        end: u64,
        /// Shortest start signal the sensor answers.
        min_start: Duration,
    }

    /// DHT11 manual V1.3, table 10, typical column.
    const DHT11: Times = Times {
        go: 13,
        rel: 83,
        reh: 87,
        low: 54,
        h0: 24,
        h1: 71,
        end: 54,
        min_start: Duration::from_millis(18),
    };
    /// AM2302 manual, typical times.
    const AM2302: Times = Times {
        go: 30,
        rel: 80,
        reh: 80,
        low: 50,
        h0: 27,
        h1: 70,
        end: 50,
        min_start: Duration::from_millis(1),
    };

    /// The line's levels after release, as (level, delay from release).
    fn waveform(t: Times, bytes: [u8; 5]) -> Vec<(bool, Duration)> {
        let mut at = 0;
        let mut out = vec![(true, Duration::ZERO)];
        let mut push = |level, after_us: u64| {
            at += after_us;
            out.push((level, Duration::from_micros(at)));
        };
        push(false, t.go);
        push(true, t.rel);
        push(false, t.reh);
        for i in 0..40 {
            let one = bytes[i / 8] & (0x80 >> (i % 8)) != 0;
            push(true, t.low);
            push(false, if one { t.h1 } else { t.h0 });
        }
        push(true, t.end);
        out
    }

    /// A sensor that answers each start signal of at least its minimum
    /// length with the next of `frames`; `None` stays silent.
    fn sensor(clock: &Rc<FakeClock>, t: Times, frames: Vec<Option<[u8; 5]>>) -> FakeGpio {
        let mut g = FakeGpio::new(Rc::clone(clock));
        let low_since = Rc::new(Cell::new(None::<Instant>));
        let low = Rc::clone(&low_since);
        g.responder = Some(Box::new(move |line, high, now| {
            if line == LINE && !high {
                low.set(Some(now));
            }
            vec![]
        }));
        let mut frames = frames.into_iter();
        g.on_configure = Some(Box::new(move |line, c, now| {
            if line != LINE || c.direction != Direction::Input {
                return vec![];
            }
            let Some(since) = low_since.take() else {
                return vec![];
            };
            if now.saturating_duration_since(since) < t.min_start {
                return vec![(line, true, Duration::ZERO)];
            }
            match frames.next().flatten() {
                Some(b) => waveform(t, b)
                    .into_iter()
                    .map(|(level, d)| (line, level, d))
                    .collect(),
                None => vec![(line, true, Duration::ZERO)],
            }
        }));
        g
    }

    fn dht(clock: &Rc<FakeClock>, model: Model, frames: Vec<Option<[u8; 5]>>) -> Dht<FakeGpio> {
        let t = match model {
            Model::Dht11 => DHT11,
            Model::Dht22 => AM2302,
        };
        Dht::new(sensor(clock, t, frames), LINE, model, 11, 1)
    }

    /// The checksum byte for four data bytes, computed independently.
    fn with_sum(b: [u8; 4]) -> [u8; 5] {
        let s = (b.iter().map(|&x| u32::from(x)).sum::<u32>() % 256) as u8;
        [b[0], b[1], b[2], b[3], s]
    }

    fn values(s: &Sample) -> (i32, i32) {
        let ch = s.channels().unwrap();
        (ch[0].value, ch[1].value)
    }

    #[test]
    fn the_dht11_manuals_example_reads_53_percent_and_24_degrees() {
        // Example 1 of the manual: 0011 0101, 0, 0001 1000, 0, 0100 1101.
        let clock = Rc::new(FakeClock::new());
        let mut d = dht(&clock, Model::Dht11, vec![Some([0x35, 0, 0x18, 0, 0x4d])]);
        let s = d.read(&*clock).unwrap();
        assert_eq!(values(&s), (530, 240));
        let ch = s.channels().unwrap();
        assert_eq!((ch[0].raw, ch[1].raw), (0x3500, 0x1800));
        assert_eq!((ch[0].unit, ch[0].exp), (Unit::PercentRh as u16, -1));
        assert_eq!((ch[1].unit, ch[1].exp), (Unit::DegC as u16, -1));
        assert_eq!(s.kind(), Some(Kind::Humidity));
        assert_eq!(s.ts_source(), Some(TsSource::Interrupt));
        assert_eq!(d.last_frame().edges, 42);
    }

    #[test]
    fn the_dht11_manuals_bad_example_fails_the_checksum() {
        // Example 2: the same data with check byte 0100 1001.
        let clock = Rc::new(FakeClock::new());
        let mut d = dht(&clock, Model::Dht11, vec![Some([0x35, 0, 0x18, 0, 0x49])]);
        assert_eq!(d.read(&*clock), Err(Error::Checksum));
    }

    #[test]
    fn the_am2302_manuals_examples() {
        // 0000 0010 1000 1100 = 652 -> 65.2 %RH; 0000 0001 0101 1111 = 351
        // -> 35.1 degC; check byte 1110 1110.
        let clock = Rc::new(FakeClock::new());
        let mut d = dht(
            &clock,
            Model::Dht22,
            vec![
                Some([0x02, 0x8c, 0x01, 0x5f, 0xee]),
                // 1000 0000 0110 0101 is minus 10.1 degC.
                Some(with_sum([0x02, 0x8c, 0x80, 0x65])),
            ],
        );
        let s = d.read(&*clock).unwrap();
        assert_eq!(values(&s), (652, 351));
        assert!(!s.flags().contains(Flags::STALE), "not said of the AM2302");
        assert_eq!(values(&d.read(&*clock).unwrap()), (652, -101));
    }

    #[test]
    fn a_negative_dht11_temperature() {
        // V1.3: the top bit of the decimal byte marks a negative value.
        let clock = Rc::new(FakeClock::new());
        let mut d = dht(&clock, Model::Dht11, vec![Some(with_sum([40, 0, 5, 0x83]))]);
        assert_eq!(values(&d.read(&*clock).unwrap()), (400, -53));
    }

    #[test]
    fn the_start_signal_follows_the_model() {
        // The fake answers only a start signal of at least its model's
        // minimum (18 ms, 1 ms), so a reading shows the lower bound.
        for model in [Model::Dht11, Model::Dht22] {
            let clock = Rc::new(FakeClock::new());
            let mut d = dht(&clock, model, vec![Some(with_sum([1, 0, 1, 0]))]);
            assert!(d.read(&*clock).is_ok(), "{model:?}");
            assert_eq!(d.release().log.len(), 1, "{model:?}: one drive low");
        }
        // DHT11 text: at most 30 ms.
        assert!(Model::Dht11.start_signal() <= Duration::from_millis(30));
    }

    #[test]
    fn reads_are_at_least_2_s_apart() {
        let clock = Rc::new(FakeClock::new());
        let frame = Some(with_sum([50, 0, 22, 0]));
        let mut d = dht(&clock, Model::Dht11, vec![frame, frame]);
        d.read(&*clock).unwrap();
        d.read(&*clock).unwrap();
        let log = &d.release().log;
        assert!(log[1].2 - log[0].2 >= 2_000_000_000, "{log:?}");
    }

    #[test]
    fn the_first_dht11_reading_is_stale_and_marks_the_epoch() {
        let clock = Rc::new(FakeClock::new());
        let frame = Some(with_sum([50, 0, 22, 0]));
        let mut d = dht(&clock, Model::Dht11, vec![frame, frame]);
        let a = d.read(&*clock).unwrap();
        let b = d.read(&*clock).unwrap();
        assert!(a.flags().contains(Flags::STALE));
        assert!(a.flags().contains(Flags::NEW_EPOCH));
        assert!(!b.flags().contains(Flags::STALE));
        assert!(!b.flags().contains(Flags::NEW_EPOCH));
        assert_eq!((a.seq, b.seq, a.epoch, a.sensor_id), (0, 1, 1, 11));
    }

    #[test]
    fn a_lost_first_edge_is_tolerated() {
        // The transport arms edge detection 50 us after the release: the
        // answer's falling edge, 13 us in, is not seen.
        let clock = Rc::new(FakeClock::new());
        let mut d = dht(&clock, Model::Dht11, vec![Some(with_sum([61, 0, 23, 0]))]);
        d.gpio.arm_delay = Duration::from_micros(50);
        assert_eq!(values(&d.read(&*clock).unwrap()), (610, 230));
        assert_eq!(d.last_frame().edges, 41);
    }

    #[test]
    fn a_frame_missing_its_start_is_rejected() {
        // Armed 200 us late: the response's second fall, at 183 us, is lost
        // as well; the first bit's, at 261 us, is seen.
        let clock = Rc::new(FakeClock::new());
        let mut d = dht(&clock, Model::Dht11, vec![Some(with_sum([61, 0, 23, 0]))]);
        d.gpio.arm_delay = Duration::from_micros(200);
        assert_eq!(d.read(&*clock), Err(Error::EdgeCount(40)));
    }

    #[test]
    fn a_glitch_inside_the_frame_is_rejected() {
        let clock = Rc::new(FakeClock::new());
        let mut d = dht(&clock, Model::Dht11, vec![Some(with_sum([61, 0, 23, 0]))]);
        d.init().unwrap();
        // A 2 us spike low inside the response's high.
        let gpio = &mut d.gpio;
        let mut hook = gpio.on_configure.take().unwrap();
        gpio.on_configure = Some(Box::new(move |line, c, now| {
            let mut edges = hook(line, c, now);
            if c.direction == Direction::Input && edges.len() > 1 {
                let rise = edges[2].2;
                edges.push((line, false, rise + Duration::from_micros(10)));
                edges.push((line, true, rise + Duration::from_micros(12)));
                edges.sort_by_key(|e| e.2);
            }
            edges
        }));
        assert_eq!(d.read(&*clock), Err(Error::EdgeCount(43)));
    }

    #[test]
    fn a_frame_shifted_by_one_edge_fails_on_the_response() {
        // The answer's first fall seen, one data fall lost: 41 edges, but
        // the first period is the response, longer than any bit.
        let times: Vec<i64> = {
            let w = waveform(DHT11, with_sum([61, 0, 23, 0]));
            let mut falls: Vec<i64> = w
                .iter()
                .filter(|(level, _)| !level)
                .map(|(_, d)| d.as_nanos() as i64)
                .collect();
            falls.remove(20);
            falls
        };
        assert_eq!(times.len(), 41);
        let mut info = FrameInfo::default();
        assert_eq!(decode(&times, &mut info), Err(Error::Timing { bit: 0 }));
    }

    #[test]
    fn a_response_of_the_wrong_length_is_rejected() {
        let clock = Rc::new(FakeClock::new());
        let slow = Times { reh: 120, ..DHT11 };
        let mut d = Dht::new(
            sensor(&clock, slow, vec![Some(with_sum([61, 0, 23, 0]))]),
            LINE,
            Model::Dht11,
            11,
            1,
        );
        assert_eq!(d.read(&*clock), Err(Error::Timing { bit: 40 }));
    }

    #[test]
    fn a_bit_period_outside_the_window_is_rejected() {
        // A 1 whose high lasts 100 us: period 154 us, past 132 + 12.
        let clock = Rc::new(FakeClock::new());
        let long_one = Times { h1: 100, ..DHT11 };
        let mut d = Dht::new(
            sensor(&clock, long_one, vec![Some(with_sum([0x80, 0, 0, 0]))]),
            LINE,
            Model::Dht11,
            11,
            1,
        );
        assert_eq!(d.read(&*clock), Err(Error::Timing { bit: 0 }));
    }

    #[test]
    fn the_extremes_of_the_dht11_table_decode() {
        // Every time at its minimum, then at its maximum.
        let min = Times {
            go: 10,
            rel: 78,
            reh: 80,
            low: 50,
            h0: 23,
            h1: 68,
            end: 52,
            ..DHT11
        };
        let max = Times {
            go: 35,
            rel: 88,
            reh: 92,
            low: 58,
            h0: 27,
            h1: 74,
            end: 56,
            ..DHT11
        };
        // Alternating bits: 0101 0101 and 0010 1010.
        let frame = with_sum([0x55, 0, 0x2a, 0x05]);
        for t in [min, max] {
            let clock = Rc::new(FakeClock::new());
            let mut d = Dht::new(sensor(&clock, t, vec![Some(frame)]), LINE, Model::Dht11, 11, 1);
            assert_eq!(values(&d.read(&*clock).unwrap()), (850, 425));
        }
    }

    #[test]
    fn no_sensor_is_no_response() {
        let clock = Rc::new(FakeClock::new());
        let mut d = dht(&clock, Model::Dht22, vec![None]);
        assert_eq!(d.read(&*clock), Err(Error::NoResponse));
        assert_eq!(d.last_frame().edges, 0);
    }

    #[test]
    fn a_short_start_signal_gets_no_answer() {
        // A DHT11 model that needs 18 ms, read as a DHT22 (10 ms).
        let clock = Rc::new(FakeClock::new());
        let mut d = Dht::new(
            sensor(&clock, DHT11, vec![Some(with_sum([50, 0, 22, 0]))]),
            LINE,
            Model::Dht22,
            11,
            1,
        );
        assert_eq!(d.read(&*clock), Err(Error::NoResponse));
    }

    #[test]
    fn the_wrong_model_is_implausible() {
        // A DHT22 frame read as a DHT11: a non-zero humidity decimal.
        let clock = Rc::new(FakeClock::new());
        let mut d = dht(&clock, Model::Dht11, vec![Some([0x02, 0x8c, 0x01, 0x5f, 0xee])]);
        assert_eq!(d.read(&*clock), Err(Error::Implausible));
        // A DHT11 frame read as a DHT22: 0x3500 is 1356.8 %RH.
        let clock = Rc::new(FakeClock::new());
        let mut d = Dht::new(
            sensor(&clock, AM2302, vec![Some([0x35, 0, 0x18, 0, 0x4d])]),
            LINE,
            Model::Dht22,
            11,
            1,
        );
        assert_eq!(d.read(&*clock), Err(Error::Implausible));
    }

    #[test]
    fn dht11_decimals_above_nine_are_implausible() {
        assert_eq!(convert(Model::Dht11, with_sum([40, 0, 21, 9])), Some((400, 219)));
        assert_eq!(convert(Model::Dht11, with_sum([40, 0, 21, 10])), None);
        // V1.3: the humidity decimal is 0.
        assert_eq!(convert(Model::Dht11, with_sum([40, 1, 21, 0])), None);
        assert_eq!(convert(Model::Dht11, with_sum([101, 0, 21, 0])), None);
        assert_eq!(convert(Model::Dht22, with_sum([0x03, 0xe8, 0, 0])), Some((1000, 0)));
        assert_eq!(convert(Model::Dht22, with_sum([0x03, 0xe9, 0, 0])), None);
    }

    #[test]
    fn values_outside_the_range_are_saturated() {
        let clock = Rc::new(FakeClock::new());
        let mut d = dht(
            &clock,
            Model::Dht11,
            vec![Some(with_sum([97, 0, 25, 0])), Some(with_sum([60, 0, 25, 0]))],
        );
        assert!(d.read(&*clock).unwrap().flags().contains(Flags::SATURATED));
        assert!(!d.read(&*clock).unwrap().flags().contains(Flags::SATURATED));
        let clock = Rc::new(FakeClock::new());
        let mut d = dht(&clock, Model::Dht22, vec![Some(with_sum([0x01, 0, 0x83, 0x52]))]);
        let s = d.read(&*clock).unwrap();
        assert_eq!(values(&s), (256, -850));
        assert!(s.flags().contains(Flags::SATURATED));
    }

    #[test]
    fn stale_edges_before_the_start_are_ignored() {
        let clock = Rc::new(FakeClock::new());
        let mut d = dht(&clock, Model::Dht11, vec![Some(with_sum([45, 0, 19, 0]))]);
        d.init().unwrap();
        d.gpio.schedule(LINE, false, Duration::from_micros(5));
        d.gpio.schedule(LINE, true, Duration::from_micros(9));
        clock.advance(Duration::from_millis(1));
        assert_eq!(values(&d.read(&*clock).unwrap()), (450, 190));
    }

    #[test]
    fn lost_edges_during_the_frame_are_an_error() {
        let clock = Rc::new(FakeClock::new());
        let mut d = dht(&clock, Model::Dht11, vec![Some(with_sum([45, 0, 19, 0]))]);
        // The drain's wait runs normally, the first wait in the frame fails.
        d.gpio.outcomes.extend([None, Some(BusError::Overflow)]);
        assert_eq!(d.read(&*clock), Err(Error::Bus(BusError::Overflow)));
    }

    #[test]
    fn a_line_that_never_settles_is_noisy() {
        let clock = Rc::new(FakeClock::new());
        let mut d = dht(&clock, Model::Dht11, vec![]);
        d.gpio.outcomes.extend([Some(BusError::Overflow); DRAIN_LIMIT]);
        assert_eq!(d.read(&*clock), Err(Error::Noisy));
    }

    #[test]
    fn frame_info_reports_what_was_seen() {
        let clock = Rc::new(FakeClock::new());
        let mut d = dht(&clock, Model::Dht22, vec![Some(with_sum([0x02, 0x8c, 0x01, 0x5f]))]);
        d.read(&*clock).unwrap();
        let f = d.last_frame();
        assert_eq!(f.edges, 42);
        assert_eq!(f.first_edge_ns, 30_000);
        // AM2302 typical: 0 is 50 + 27 us, 1 is 50 + 70 us.
        assert_eq!(f.zero_ns, (77_000, 77_000));
        assert_eq!(f.one_ns, (120_000, 120_000));
    }
}
