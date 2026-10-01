// SPDX-License-Identifier: BSD-3-Clause
//! The five primitives: time, GPIO, I2C, 1-Wire and SPI.
//!
//! Every blocking operation takes an absolute deadline on the monotonic clock.
//! A driver never waits without one and never retries on its own; retry
//! policy belongs to the caller. There is no ADC primitive (an ADC is a device
//! on a bus) and no pulse primitive (a pulse is two timestamped edges).

use core::time::Duration;

/// A point on the transport's monotonic clock, in nanoseconds. Wall-clock
/// time is never used.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Instant(i64);

impl Instant {
    pub const fn from_nanos(ns: i64) -> Self {
        Instant(ns)
    }

    pub const fn as_nanos(self) -> i64 {
        self.0
    }

    /// `None` if the result does not fit the clock's range.
    pub fn checked_add(self, d: Duration) -> Option<Self> {
        let ns = i64::try_from(d.as_nanos()).ok()?;
        self.0.checked_add(ns).map(Instant)
    }

    /// Zero if `earlier` is later than `self`.
    pub fn saturating_duration_since(self, earlier: Instant) -> Duration {
        let ns = self.0.saturating_sub(earlier.0);
        Duration::from_nanos(u64::try_from(ns).unwrap_or(0))
    }
}

/// Why a bus operation failed. The discriminants are stable because the
/// value may cross a component boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum BusError {
    /// The deadline passed.
    Timeout = 1,
    /// An I2C address or data byte was not acknowledged.
    Nak = 2,
    /// A 1-Wire reset saw no device.
    NoPresence = 3,
    /// Transport-level failure.
    Io = 4,
    /// Bad argument, such as an I2C address above 0x7f.
    Invalid = 5,
    /// Edge events were lost before they were read.
    Overflow = 6,
}

/// Where a timestamp was taken.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum TsSource {
    /// When the driver read the value back.
    DriverRead = 0,
    /// In the interrupt path.
    Interrupt = 1,
    /// Latched by bus or timer hardware.
    Hardware = 2,
}

impl TsSource {
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(TsSource::DriverRead),
            1 => Some(TsSource::Interrupt),
            2 => Some(TsSource::Hardware),
            _ => None,
        }
    }
}

/// Monotonic time and a coarse delay.
pub trait Clock {
    fn now(&self) -> Instant;
    fn sleep_until(&self, t: Instant);
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Input,
    Output,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Bias {
    None,
    PullUp,
    PullDown,
}

/// Which edges on an input line produce events.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Edges {
    pub rising: bool,
    pub falling: bool,
}

impl Edges {
    pub const NONE: Edges = Edges {
        rising: false,
        falling: false,
    };
    pub const RISING: Edges = Edges {
        rising: true,
        falling: false,
    };
    pub const FALLING: Edges = Edges {
        rising: false,
        falling: true,
    };
    pub const BOTH: Edges = Edges {
        rising: true,
        falling: true,
    };
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LineConfig {
    pub direction: Direction,
    pub bias: Bias,
    pub edges: Edges,
}

/// One edge on a configured line. Fixed layout: a transport in another
/// protection domain may deliver these through shared memory.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(C, align(8))]
pub struct GpioEvent {
    /// [`Instant::as_nanos`] of the edge.
    pub timestamp: i64,
    pub line: u32,
    /// Line level after the edge: 0 or 1.
    pub level: u8,
    /// A [`TsSource`] value.
    pub ts_source: u8,
    pub reserved: u16,
}

const _: () = assert!(core::mem::size_of::<GpioEvent>() == 16);
const _: () = assert!(core::mem::align_of::<GpioEvent>() == 8);

/// Levels and timestamped edges on the lines of one GPIO controller.
pub trait Gpio {
    fn configure(&mut self, line: u32, config: LineConfig) -> Result<(), BusError>;
    fn set(&mut self, line: u32, high: bool) -> Result<(), BusError>;
    fn get(&mut self, line: u32) -> Result<bool, BusError>;

    /// Fills `out` with pending edge events, oldest first, and returns how
    /// many. Waits until at least one is pending or the deadline passes
    /// ([`BusError::Timeout`]). [`BusError::Overflow`] means events were lost
    /// since the last call; the caller must resynchronise.
    fn wait_edges(&mut self, out: &mut [GpioEvent], deadline: Instant) -> Result<usize, BusError>;
}

/// One I2C bus.
pub trait I2c {
    /// Writes `write` to the 7-bit address `addr`, then, if `read` is not
    /// empty, reads into it after a repeated start.
    ///
    /// Both empty is [`BusError::Invalid`]. An address-only probe would need
    /// a zero-length message, which some adapters refuse, so the contract
    /// does not offer one; answering `Ok` without touching the bus would
    /// report a device that may not be there.
    fn transfer(
        &mut self,
        addr: u8,
        write: &[u8],
        read: &mut [u8],
        deadline: Instant,
    ) -> Result<(), BusError>;
}

/// One 1-Wire bus. There is no ROM search, so a bus holds one device until
/// a search primitive is justified.
pub trait OneWire {
    /// Reset and presence pulse, then write `write`, then read into `read`.
    fn transaction(
        &mut self,
        write: &[u8],
        read: &mut [u8],
        deadline: Instant,
    ) -> Result<(), BusError>;
}

/// One SPI device (bus and chip select). Not used by the sensor kit;
/// declared so that a later board does not force a second seam.
pub trait Spi {
    /// Full duplex. `tx` and `rx` have equal length, or one of them is empty.
    fn transfer(&mut self, tx: &[u8], rx: &mut [u8], deadline: Instant) -> Result<(), BusError>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn instant_arithmetic_saturates_or_refuses() {
        let t = Instant::from_nanos(1_000);
        assert_eq!(
            t.checked_add(Duration::from_nanos(500)),
            Some(Instant::from_nanos(1_500))
        );
        assert_eq!(
            Instant::from_nanos(i64::MAX).checked_add(Duration::from_nanos(1)),
            None
        );
        assert_eq!(
            t.saturating_duration_since(Instant::from_nanos(2_000)),
            Duration::ZERO
        );
        assert_eq!(
            t.saturating_duration_since(Instant::from_nanos(400)),
            Duration::from_nanos(600)
        );
    }

    #[test]
    fn ts_source_decodes_only_known_values() {
        assert_eq!(TsSource::from_u8(2), Some(TsSource::Hardware));
        assert_eq!(TsSource::from_u8(3), None);
    }
}
