// SPDX-License-Identifier: BSD-3-Clause
//! NXP PCF8591: four-channel 8-bit A/D converter with one D/A output, on I2C.
//!
//! Follows the NXP PCF8591 datasheet (Rev. 6): the control byte selects the
//! input, and the first byte of every read is the result of the *previous*
//! conversion (0x80 after power-on). Each channel is therefore read as its own
//! two-byte transfer and the second byte kept. The auto-increment mode is not
//! used, which keeps each value unambiguous at the cost of four transfers.
//!
//! The chip has no identification register, so a wrong device at the address
//! cannot be detected; only a missing one (NAK).

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::time::Duration;

use sensor_core::bus::{BusError, Clock, I2c, Instant, TsSource};
use sensor_core::sample::{Channel, Flags, Kind, MAX_CHANNELS, Sample, Unit};

/// Address with A2..A0 tied low; the range is 0x48 to 0x4F.
pub const BASE_ADDR: u8 = 0x48;
pub const INPUTS: usize = 4;

/// Four single-ended inputs, analog output off, no auto-increment.
const CTRL_SINGLE_ENDED: u8 = 0x00;
const TRANSFER: Duration = Duration::from_millis(50);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Bus(BusError),
    /// The clock cannot represent the deadline.
    Clock,
}

impl From<BusError> for Error {
    fn from(e: BusError) -> Self {
        Error::Bus(e)
    }
}

pub struct Pcf8591<I> {
    i2c: I,
    addr: u8,
    vref_mv: u16,
    sensor_id: u32,
    epoch: u32,
    seq: u32,
    announce_epoch: bool,
}

impl<I: I2c> Pcf8591<I> {
    /// `vref_mv` is the reference voltage in millivolts; on modules that tie
    /// VREF to VCC it is the supply voltage.
    pub fn new(i2c: I, addr: u8, vref_mv: u16, sensor_id: u32, epoch: u32) -> Self {
        Pcf8591 {
            i2c,
            addr,
            vref_mv,
            sensor_id,
            epoch,
            seq: 0,
            announce_epoch: true,
        }
    }

    pub fn release(self) -> I {
        self.i2c
    }

    /// Converts AIN0 to AIN3. Each channel's raw value is the 8-bit code and
    /// its scaled value is millivolts: code * VREF / 256.
    ///
    /// The channels are not simultaneous. A conversion starts on the
    /// acknowledge of the read address, so input n is converted during
    /// transfer n, and the one timestamp is taken after the fourth. AIN0 is
    /// therefore three transfers older than AIN3. A transfer is about 48 bit
    /// times, so at 100 kHz that is at least 1.4 ms, plus the driver's
    /// per-transfer overhead, which has not been measured.
    pub fn read<C: Clock>(&mut self, clock: &C) -> Result<Sample, Error> {
        let mut ch = [Channel::default(); MAX_CHANNELS];
        for (n, c) in ch.iter_mut().take(INPUTS).enumerate() {
            let mut buf = [0u8; 2];
            let ctrl = CTRL_SINGLE_ENDED | n as u8;
            self.i2c
                .transfer(self.addr, &[ctrl], &mut buf, after(clock, TRANSFER)?)?;
            let code = buf[1];
            *c = Channel {
                raw: i32::from(code),
                value: (i32::from(code) * i32::from(self.vref_mv)) / 256,
                unit: Unit::Volt as u16,
                exp: -3,
                reserved: 0,
            };
        }
        let timestamp = clock.now();
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
            kind: Kind::Analog as u16,
            flags: flags.0,
            ts_source: TsSource::DriverRead as u8,
            nchannels: INPUTS as u8,
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
    use sensor_fake::{FakeClock, FakeDevice, FakeI2c};

    /// The datasheet's read behaviour: each byte returned is the previous
    /// conversion; a conversion of the selected input follows every byte.
    struct Model {
        inputs: [u8; INPUTS],
        last: u8,
        ctrl: u8,
    }

    impl FakeDevice for Model {
        fn addr(&self) -> u8 {
            BASE_ADDR
        }

        fn transfer(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), BusError> {
            if let Some(&c) = write.first() {
                self.ctrl = c;
            }
            for b in read.iter_mut() {
                *b = self.last;
                self.last = self.inputs[usize::from(self.ctrl & 0b11)];
                if self.ctrl & 0x04 != 0 {
                    self.ctrl = (self.ctrl & !0b11) | ((self.ctrl + 1) & 0b11);
                }
            }
            Ok(())
        }
    }

    fn model(inputs: [u8; INPUTS]) -> FakeI2c {
        FakeI2c::with_device(Model {
            inputs,
            last: 0x80,
            ctrl: 0,
        })
    }

    #[test]
    fn each_input_is_read_without_the_stale_byte() {
        let mut adc = Pcf8591::new(model([0, 64, 128, 255]), BASE_ADDR, 3300, 33, 1);
        let s = adc.read(&FakeClock::new()).unwrap();
        let ch = s.channels().unwrap();
        assert_eq!(
            ch.iter().map(|c| c.raw).collect::<Vec<_>>(),
            [0, 64, 128, 255]
        );
        assert_eq!(
            ch.iter().map(|c| c.value).collect::<Vec<_>>(),
            [0, 825, 1650, 3287]
        );
        assert_eq!(s.kind(), Some(Kind::Analog));
    }

    #[test]
    fn inputs_are_selected_one_by_one() {
        let mut adc = Pcf8591::new(model([1, 2, 3, 4]), BASE_ADDR, 3300, 33, 1);
        adc.read(&FakeClock::new()).unwrap();
        let writes: Vec<_> = adc
            .release()
            .log
            .iter()
            .map(|t| (t.write.clone(), t.read_len))
            .collect();
        assert_eq!(
            writes,
            [(vec![0], 2), (vec![1], 2), (vec![2], 2), (vec![3], 2)]
        );
    }

    #[test]
    fn a_missing_chip_is_a_nak() {
        let mut adc = Pcf8591::new(model([0; 4]), BASE_ADDR + 1, 3300, 33, 1);
        assert_eq!(adc.read(&FakeClock::new()), Err(Error::Bus(BusError::Nak)));
    }

    #[test]
    fn only_the_first_sample_opens_the_epoch() {
        let clock = FakeClock::new();
        let mut adc = Pcf8591::new(model([0; 4]), BASE_ADDR, 3300, 33, 4);
        let a = adc.read(&clock).unwrap();
        let b = adc.read(&clock).unwrap();
        assert!(a.flags().contains(Flags::NEW_EPOCH) && !b.flags().contains(Flags::NEW_EPOCH));
        assert_eq!((a.seq, b.seq, b.epoch), (0, 1, 4));
    }
}
