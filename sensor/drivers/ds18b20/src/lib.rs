// SPDX-License-Identifier: BSD-3-Clause
//! Maxim DS18B20 digital thermometer, on a 1-Wire bus with one device.
//!
//! Follows the Maxim DS18B20 datasheet (19-7487): Skip ROM (0xCC) addresses
//! the only device, Convert T (0x44) starts a conversion, Read Scratchpad
//! (0xBE) returns nine bytes whose last is a CRC-8 over the first eight
//! (polynomial x^8 + x^5 + x^4 + 1). The conversion time depends on the
//! resolution in configuration byte 4: 93.75, 187.5, 375 or 750 ms for 9 to
//! 12 bits. The driver waits the full time rather than polling, because a
//! transaction always begins with a reset, which would interrupt a conversion.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::time::Duration;

use sensor_core::bus::{BusError, Clock, Instant, OneWire, TsSource};
use sensor_core::sample::{Channel, Flags, Kind, MAX_CHANNELS, Sample, Unit};

pub const FAMILY_CODE: u8 = 0x28;

const SKIP_ROM: u8 = 0xCC;
const CONVERT_T: u8 = 0x44;
const READ_SCRATCHPAD: u8 = 0xBE;
const SCRATCHPAD_LEN: usize = 9;
const TRANSFER: Duration = Duration::from_millis(100);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Bus(BusError),
    /// The scratchpad failed its CRC.
    Crc,
    /// The scratchpad read back as all ones: nothing drove the bus.
    NoData,
    /// The clock cannot represent the deadline.
    Clock,
}

impl From<BusError> for Error {
    fn from(e: BusError) -> Self {
        Error::Bus(e)
    }
}

/// Conversion resolution, from configuration register bits R1 and R0.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Resolution {
    Bits9,
    Bits10,
    Bits11,
    Bits12,
}

impl Resolution {
    fn from_config(config: u8) -> Self {
        match (config >> 5) & 0b11 {
            0 => Resolution::Bits9,
            1 => Resolution::Bits10,
            2 => Resolution::Bits11,
            _ => Resolution::Bits12,
        }
    }

    /// Maximum conversion time (datasheet, AC electrical characteristics).
    pub fn conversion_time(self) -> Duration {
        Duration::from_micros(match self {
            Resolution::Bits9 => 93_750,
            Resolution::Bits10 => 187_500,
            Resolution::Bits11 => 375_000,
            Resolution::Bits12 => 750_000,
        })
    }

    /// Low temperature bits that are undefined at this resolution.
    fn undefined_bits(self) -> i16 {
        match self {
            Resolution::Bits9 => 0b111,
            Resolution::Bits10 => 0b11,
            Resolution::Bits11 => 0b1,
            Resolution::Bits12 => 0,
        }
    }
}

/// Dallas/Maxim CRC-8, reflected polynomial 0x8C.
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &byte in data {
        let mut b = byte;
        for _ in 0..8 {
            let mix = (crc ^ b) & 1;
            crc >>= 1;
            if mix != 0 {
                crc ^= 0x8C;
            }
            b >>= 1;
        }
    }
    crc
}

pub struct Ds18b20<W> {
    bus: W,
    sensor_id: u32,
    epoch: u32,
    seq: u32,
    /// Until the first scratchpad read, assume the power-on default.
    resolution: Resolution,
    announce_epoch: bool,
}

impl<W: OneWire> Ds18b20<W> {
    pub fn new(bus: W, sensor_id: u32, epoch: u32) -> Self {
        Ds18b20 {
            bus,
            sensor_id,
            epoch,
            seq: 0,
            resolution: Resolution::Bits12,
            announce_epoch: true,
        }
    }

    pub fn release(self) -> W {
        self.bus
    }

    pub fn resolution(&self) -> Resolution {
        self.resolution
    }

    /// One conversion. The channel's raw value is the temperature register
    /// in 1/16 degC; its scaled value is degC with four decimals.
    pub fn read<C: Clock>(&mut self, clock: &C) -> Result<Sample, Error> {
        self.bus
            .transaction(&[SKIP_ROM, CONVERT_T], &mut [], after(clock, TRANSFER)?)?;
        clock.sleep_until(after(clock, self.resolution.conversion_time())?);
        let mut sp = [0u8; SCRATCHPAD_LEN];
        self.bus.transaction(
            &[SKIP_ROM, READ_SCRATCHPAD],
            &mut sp,
            after(clock, TRANSFER)?,
        )?;
        let timestamp = clock.now();
        if sp.iter().all(|&b| b == 0xFF) {
            return Err(Error::NoData);
        }
        if crc8(&sp[..8]) != sp[8] {
            return Err(Error::Crc);
        }
        self.resolution = Resolution::from_config(sp[4]);
        let raw = i16::from_le_bytes([sp[0], sp[1]]) & !self.resolution.undefined_bits();

        let mut ch = [Channel::default(); MAX_CHANNELS];
        ch[0] = Channel {
            raw: i32::from(raw),
            value: i32::from(raw) * 625,
            unit: Unit::DegC as u16,
            exp: -4,
            reserved: 0,
        };
        let flags = if self.announce_epoch {
            Flags::NEW_EPOCH
        } else {
            Flags::default()
        };
        let sample = Sample {
            timestamp: timestamp.as_nanos(),
            sensor_id: self.sensor_id,
            seq: self.seq,
            epoch: self.epoch,
            kind: Kind::Temperature as u16,
            flags: flags.0,
            ts_source: TsSource::DriverRead as u8,
            nchannels: 1,
            ch,
            ..Sample::default()
        };
        self.announce_epoch = false;
        self.seq = self.seq.wrapping_add(1);
        Ok(sample)
    }
}

fn after<C: Clock>(clock: &C, d: Duration) -> Result<Instant, Error> {
    clock.now().checked_add(d).ok_or(Error::Clock)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sensor_fake::{FakeClock, FakeOneWire};

    /// Scratchpad read from a DS18B20 module on 2026-09-30: 0x018F is
    /// 24.9375 degC, configuration 0x7F is 12-bit, CRC 0x1A.
    const MODULE_SCRATCHPAD: [u8; 9] = [0x8f, 0x01, 0x55, 0x05, 0x7f, 0xa5, 0xa5, 0x66, 0x1a];

    fn device(scratchpad: [u8; 9]) -> FakeOneWire {
        FakeOneWire::with_device(Box::new(move |write, read| {
            if write == [SKIP_ROM, READ_SCRATCHPAD] {
                read.copy_from_slice(&scratchpad);
            }
            Ok(())
        }))
    }

    /// A scratchpad with a valid CRC for the given temperature and config.
    fn scratchpad(raw: i16, config: u8) -> [u8; 9] {
        let [lo, hi] = raw.to_le_bytes();
        let mut sp = [lo, hi, 0x4b, 0x46, config, 0xff, 0x0c, 0x10, 0];
        sp[8] = crc8(&sp[..8]);
        sp
    }

    #[test]
    fn crc_matches_the_module_scratchpad() {
        assert_eq!(crc8(&MODULE_SCRATCHPAD[..8]), 0x1a);
        // Any message followed by its own CRC checks to zero.
        assert_eq!(crc8(&MODULE_SCRATCHPAD), 0);
    }

    #[test]
    fn module_reading_is_24_9375_degc() {
        let clock = FakeClock::new();
        let mut t = Ds18b20::new(device(MODULE_SCRATCHPAD), 23, 1);
        let s = t.read(&clock).unwrap();
        let ch = s.channels().unwrap();
        assert_eq!((ch[0].raw, ch[0].value, ch[0].exp), (399, 249_375, -4));
        assert_eq!(s.kind(), Some(Kind::Temperature));
    }

    #[test]
    fn the_first_read_waits_the_12_bit_conversion_time() {
        let clock = FakeClock::new();
        let mut t = Ds18b20::new(device(MODULE_SCRATCHPAD), 23, 1);
        t.read(&clock).unwrap();
        assert!(clock.now().as_nanos() >= 750_000_000);
        let log = t.release().log;
        assert_eq!(log, [(vec![0xCC, 0x44], 0), (vec![0xCC, 0xBE], 9)]);
    }

    #[test]
    fn a_lower_resolution_shortens_the_wait_and_masks_bits() {
        let clock = FakeClock::new();
        // 9-bit configuration; the three lowest bits are undefined.
        let mut t = Ds18b20::new(device(scratchpad(0x0197, 0x1f)), 23, 1);
        let first = t.read(&clock).unwrap();
        assert_eq!(first.channels().unwrap()[0].raw, 0x0190);
        assert_eq!(t.resolution(), Resolution::Bits9);
        let before = clock.now();
        t.read(&clock).unwrap();
        let waited = clock.now().saturating_duration_since(before);
        assert!(waited >= Duration::from_micros(93_750) && waited < Duration::from_millis(100));
    }

    #[test]
    fn negative_temperatures_keep_their_sign() {
        // Datasheet table 1: 0xFE6F is -25.0625 degC.
        let mut t = Ds18b20::new(device(scratchpad(-401, 0x7f)), 23, 1);
        let s = t.read(&FakeClock::new()).unwrap();
        assert_eq!(s.channels().unwrap()[0].value, -250_625);
    }

    #[test]
    fn a_corrupt_scratchpad_is_refused() {
        let mut sp = MODULE_SCRATCHPAD;
        sp[0] ^= 1;
        let mut t = Ds18b20::new(device(sp), 23, 1);
        assert_eq!(t.read(&FakeClock::new()), Err(Error::Crc));
    }

    #[test]
    fn an_undriven_bus_is_not_data() {
        let mut t = Ds18b20::new(device([0xff; 9]), 23, 1);
        assert_eq!(t.read(&FakeClock::new()), Err(Error::NoData));
    }

    #[test]
    fn no_presence_pulse_is_reported() {
        let mut t = Ds18b20::new(FakeOneWire::default(), 23, 1);
        assert_eq!(
            t.read(&FakeClock::new()),
            Err(Error::Bus(BusError::NoPresence))
        );
    }
}
