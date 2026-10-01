// SPDX-License-Identifier: BSD-3-Clause
//! What a sensor driver hands upward: fixed-size integer records.
//!
//! Records hold no pointers and no padding, so they can be copied into a
//! shared ring buffer and read by a component that does not trust the
//! producer. Consumers still bounds-check every field: [`Sample::channels`]
//! refuses a channel count above [`MAX_CHANNELS`], and registry values a
//! consumer does not know decode to `None`. No floating point: a scaled value
//! is `value * 10^exp` in `unit`.
//!
//! Byte encoding is native-endian. A record crosses protection domains on one
//! machine, not the network.

use crate::bus::{Instant, TsSource};

pub const MAX_CHANNELS: usize = 8;
pub const SAMPLE_SIZE: usize = 128;
pub const EVENT_SIZE: usize = 32;

/// What a sample measures. Append-only registry: never renumber.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum Kind {
    Temperature = 1,
    Pressure = 2,
    Humidity = 3,
    Imu = 4,
    Distance = 5,
    Analog = 6,
    Light = 7,
}

impl Kind {
    pub const fn from_u16(v: u16) -> Option<Self> {
        Some(match v {
            1 => Kind::Temperature,
            2 => Kind::Pressure,
            3 => Kind::Humidity,
            4 => Kind::Imu,
            5 => Kind::Distance,
            6 => Kind::Analog,
            7 => Kind::Light,
            _ => return None,
        })
    }
}

/// Unit of a scaled channel value. Append-only registry: never renumber.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum Unit {
    None = 0,
    DegC = 1,
    Pascal = 2,
    PercentRh = 3,
    Metre = 4,
    MetrePerS2 = 5,
    RadPerS = 6,
    Volt = 7,
    Second = 8,
    Count = 9,
}

impl Unit {
    pub const fn from_u16(v: u16) -> Option<Self> {
        Some(match v {
            0 => Unit::None,
            1 => Unit::DegC,
            2 => Unit::Pascal,
            3 => Unit::PercentRh,
            4 => Unit::Metre,
            5 => Unit::MetrePerS2,
            6 => Unit::RadPerS,
            7 => Unit::Volt,
            8 => Unit::Second,
            9 => Unit::Count,
            _ => return None,
        })
    }
}

/// What an event reports. Append-only registry: never renumber.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum EventType {
    /// `value` is the new level.
    Edge = 1,
    /// `value` is the new state.
    State = 2,
    /// `value` is a signed change, such as encoder steps.
    Delta = 3,
}

impl EventType {
    pub const fn from_u16(v: u16) -> Option<Self> {
        Some(match v {
            1 => EventType::Edge,
            2 => EventType::State,
            3 => EventType::Delta,
            _ => return None,
        })
    }
}

/// Record flags.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Flags(pub u16);

impl Flags {
    /// Records were lost before this one.
    pub const SEQ_GAP: Flags = Flags(1 << 0);
    /// A repeated reading, not a new one.
    pub const STALE: Flags = Flags(1 << 1);
    /// At the sensor's range limit.
    pub const SATURATED: Flags = Flags(1 << 2);
    /// The scaled value lacks calibration.
    pub const UNCALIBRATED: Flags = Flags(1 << 3);
    /// First record after a timeout or reset.
    pub const RECOVERED: Flags = Flags(1 << 4);
    /// The driver restarted; `seq` restarts in a new `epoch`.
    pub const NEW_EPOCH: Flags = Flags(1 << 5);

    pub const fn contains(self, other: Flags) -> bool {
        self.0 & other.0 == other.0
    }

    pub const fn union(self, other: Flags) -> Flags {
        Flags(self.0 | other.0)
    }
}

/// One measured quantity.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct Channel {
    /// As read from the device.
    pub raw: i32,
    /// `value * 10^exp`, in `unit`.
    pub value: i32,
    /// A [`Unit`] value.
    pub unit: u16,
    pub exp: i8,
    pub reserved: u8,
}

/// One reading of up to [`MAX_CHANNELS`] quantities.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C, align(8))]
pub struct Sample {
    /// [`Instant::as_nanos`] of the acquisition.
    pub timestamp: i64,
    pub sensor_id: u32,
    /// Per sensor, within an epoch.
    pub seq: u32,
    /// Increments when the driver restarts.
    pub epoch: u32,
    /// A [`Kind`] value.
    pub kind: u16,
    pub flags: u16,
    /// A [`TsSource`] value.
    pub ts_source: u8,
    /// Valid entries at the start of `ch`; at most [`MAX_CHANNELS`].
    pub nchannels: u8,
    pub reserved0: u16,
    pub reserved1: u32,
    pub ch: [Channel; MAX_CHANNELS],
}

/// A discrete occurrence: an edge, a state change, a count.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C, align(8))]
pub struct Event {
    /// [`Instant::as_nanos`] of the occurrence.
    pub timestamp: i64,
    pub sensor_id: u32,
    pub seq: u32,
    pub epoch: u32,
    /// An [`EventType`] value.
    pub event_type: u16,
    pub flags: u16,
    pub value: i32,
    /// A [`TsSource`] value.
    pub ts_source: u8,
    pub reserved: [u8; 3],
}

// The layout is the contract: changing it changes the interface for every
// consumer, so it is checked at compile time on every target.
const _: () = {
    use core::mem::{align_of, offset_of, size_of};
    assert!(size_of::<Channel>() == 12);
    assert!(size_of::<Sample>() == SAMPLE_SIZE);
    assert!(offset_of!(Sample, ts_source) == 24);
    assert!(offset_of!(Sample, ch) == 32);
    assert!(size_of::<Event>() == EVENT_SIZE);
    assert!(offset_of!(Event, value) == 24);
    assert!(align_of::<Sample>() == 8 && align_of::<Event>() == 8);
};

impl Sample {
    pub fn instant(&self) -> Instant {
        Instant::from_nanos(self.timestamp)
    }

    pub fn kind(&self) -> Option<Kind> {
        Kind::from_u16(self.kind)
    }

    pub fn flags(&self) -> Flags {
        Flags(self.flags)
    }

    pub fn ts_source(&self) -> Option<TsSource> {
        TsSource::from_u8(self.ts_source)
    }

    /// The valid channels, or `None` if the producer claimed more than
    /// [`MAX_CHANNELS`].
    pub fn channels(&self) -> Option<&[Channel]> {
        self.ch.get(..usize::from(self.nchannels))
    }

    pub fn to_ne_bytes(&self) -> [u8; SAMPLE_SIZE] {
        let mut w = Writer::<SAMPLE_SIZE>::new();
        w.i64(self.timestamp);
        w.u32(self.sensor_id);
        w.u32(self.seq);
        w.u32(self.epoch);
        w.u16(self.kind);
        w.u16(self.flags);
        w.u8(self.ts_source);
        w.u8(self.nchannels);
        w.u16(self.reserved0);
        w.u32(self.reserved1);
        for c in &self.ch {
            w.i32(c.raw);
            w.i32(c.value);
            w.u16(c.unit);
            w.u8(c.exp as u8);
            w.u8(c.reserved);
        }
        w.finish()
    }

    /// Decodes any 128 bytes. The result is not validated; use the checked
    /// accessors before trusting a field.
    pub fn from_ne_bytes(b: &[u8; SAMPLE_SIZE]) -> Self {
        let mut r = Reader::new(b);
        let mut s = Sample {
            timestamp: r.i64(),
            sensor_id: r.u32(),
            seq: r.u32(),
            epoch: r.u32(),
            kind: r.u16(),
            flags: r.u16(),
            ts_source: r.u8(),
            nchannels: r.u8(),
            reserved0: r.u16(),
            reserved1: r.u32(),
            ch: [Channel::default(); MAX_CHANNELS],
        };
        for c in &mut s.ch {
            *c = Channel {
                raw: r.i32(),
                value: r.i32(),
                unit: r.u16(),
                exp: r.u8() as i8,
                reserved: r.u8(),
            };
        }
        s
    }
}

impl Event {
    pub fn instant(&self) -> Instant {
        Instant::from_nanos(self.timestamp)
    }

    pub fn event_type(&self) -> Option<EventType> {
        EventType::from_u16(self.event_type)
    }

    pub fn flags(&self) -> Flags {
        Flags(self.flags)
    }

    pub fn ts_source(&self) -> Option<TsSource> {
        TsSource::from_u8(self.ts_source)
    }

    pub fn to_ne_bytes(&self) -> [u8; EVENT_SIZE] {
        let mut w = Writer::<EVENT_SIZE>::new();
        w.i64(self.timestamp);
        w.u32(self.sensor_id);
        w.u32(self.seq);
        w.u32(self.epoch);
        w.u16(self.event_type);
        w.u16(self.flags);
        w.i32(self.value);
        w.u8(self.ts_source);
        for b in self.reserved {
            w.u8(b);
        }
        w.finish()
    }

    /// Decodes any 32 bytes. The result is not validated.
    pub fn from_ne_bytes(b: &[u8; EVENT_SIZE]) -> Self {
        let mut r = Reader::new(b);
        Event {
            timestamp: r.i64(),
            sensor_id: r.u32(),
            seq: r.u32(),
            epoch: r.u32(),
            event_type: r.u16(),
            flags: r.u16(),
            value: r.i32(),
            ts_source: r.u8(),
            reserved: [r.u8(), r.u8(), r.u8()],
        }
    }
}

struct Writer<const N: usize> {
    buf: [u8; N],
    pos: usize,
}

impl<const N: usize> Writer<N> {
    fn new() -> Self {
        Writer {
            buf: [0; N],
            pos: 0,
        }
    }

    fn put(&mut self, bytes: &[u8]) {
        self.buf[self.pos..self.pos + bytes.len()].copy_from_slice(bytes);
        self.pos += bytes.len();
    }

    fn u8(&mut self, v: u8) {
        self.put(&[v]);
    }
    fn u16(&mut self, v: u16) {
        self.put(&v.to_ne_bytes());
    }
    fn u32(&mut self, v: u32) {
        self.put(&v.to_ne_bytes());
    }
    fn i32(&mut self, v: i32) {
        self.put(&v.to_ne_bytes());
    }
    fn i64(&mut self, v: i64) {
        self.put(&v.to_ne_bytes());
    }

    fn finish(self) -> [u8; N] {
        debug_assert_eq!(self.pos, N);
        self.buf
    }
}

struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn new(buf: &'a [u8]) -> Self {
        Reader { buf, pos: 0 }
    }

    fn take<const K: usize>(&mut self) -> [u8; K] {
        let mut out = [0; K];
        out.copy_from_slice(&self.buf[self.pos..self.pos + K]);
        self.pos += K;
        out
    }

    fn u8(&mut self) -> u8 {
        self.take::<1>()[0]
    }
    fn u16(&mut self) -> u16 {
        u16::from_ne_bytes(self.take())
    }
    fn u32(&mut self) -> u32 {
        u32::from_ne_bytes(self.take())
    }
    fn i32(&mut self) -> i32 {
        i32::from_ne_bytes(self.take())
    }
    fn i64(&mut self) -> i64 {
        i64::from_ne_bytes(self.take())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Sample {
        let mut s = Sample {
            timestamp: 1_234_567_890,
            sensor_id: 7,
            seq: 42,
            epoch: 3,
            kind: Kind::Temperature as u16,
            flags: Flags::NEW_EPOCH.union(Flags::UNCALIBRATED).0,
            ts_source: TsSource::DriverRead as u8,
            nchannels: 1,
            ..Sample::default()
        };
        s.ch[0] = Channel {
            raw: 0x0191,
            value: 25_062,
            unit: Unit::DegC as u16,
            exp: -3,
            reserved: 0,
        };
        s
    }

    #[test]
    fn sample_round_trips_through_bytes() {
        let s = sample();
        assert_eq!(Sample::from_ne_bytes(&s.to_ne_bytes()), s);
    }

    #[test]
    fn byte_offsets_match_the_struct_layout() {
        let b = sample().to_ne_bytes();
        assert_eq!(&b[8..12], &7u32.to_ne_bytes());
        assert_eq!(b[25], 1);
        assert_eq!(&b[32..36], &0x0191i32.to_ne_bytes());
        assert_eq!(b[42], (-3i8) as u8);
    }

    #[test]
    fn event_round_trips_through_bytes() {
        let e = Event {
            timestamp: -5,
            sensor_id: 1,
            seq: 2,
            epoch: 3,
            event_type: EventType::Delta as u16,
            flags: 0,
            value: -17,
            ts_source: TsSource::Interrupt as u8,
            reserved: [0; 3],
        };
        let d = Event::from_ne_bytes(&e.to_ne_bytes());
        assert_eq!(d, e);
        assert_eq!(d.event_type(), Some(EventType::Delta));
    }

    #[test]
    fn channel_count_is_bounds_checked() {
        let mut s = sample();
        s.nchannels = 8;
        assert_eq!(s.channels().map(<[Channel]>::len), Some(8));
        s.nchannels = 9;
        assert_eq!(s.channels(), None);
        s.nchannels = 255;
        assert_eq!(Sample::from_ne_bytes(&s.to_ne_bytes()).channels(), None);
    }

    #[test]
    fn unknown_registry_values_decode_to_none() {
        let mut s = sample();
        s.kind = 999;
        s.ts_source = 9;
        assert_eq!(s.kind(), None);
        assert_eq!(s.ts_source(), None);
        assert_eq!(Unit::from_u16(10), None);
    }

    #[test]
    fn flags_combine() {
        let f = sample().flags();
        assert!(f.contains(Flags::NEW_EPOCH));
        assert!(f.contains(Flags::UNCALIBRATED));
        assert!(!f.contains(Flags::STALE));
    }
}
