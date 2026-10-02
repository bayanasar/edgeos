// SPDX-License-Identifier: BSD-3-Clause
//! Bosch BMP280 barometric pressure and temperature sensor, over I2C.
//!
//! Register map, timing and compensation follow the Bosch BMP280 datasheet
//! (BST-BMP280-DS001): memory map in section 4, measurement time in
//! section 3.8.1, compensation in section 3.11.3 (the 32-bit temperature and
//! 64-bit pressure integer formulas). Each [`Bmp280::read`] runs one
//! forced-mode conversion at x1 oversampling; the chip sleeps in between.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::time::Duration;

use sensor_core::bus::{BusError, Clock, I2c, Instant, TsSource};
use sensor_core::sample::{Channel, Flags, Kind, MAX_CHANNELS, Sample, Unit};

/// Address with SDO tied to GND.
pub const ADDR_SDO_LOW: u8 = 0x76;
/// Address with SDO tied to VDDIO.
pub const ADDR_SDO_HIGH: u8 = 0x77;
pub const CHIP_ID: u8 = 0x58;

const REG_CALIB: u8 = 0x88;
const REG_ID: u8 = 0xD0;
const REG_RESET: u8 = 0xE0;
const REG_STATUS: u8 = 0xF3;
const REG_CTRL_MEAS: u8 = 0xF4;
const REG_CONFIG: u8 = 0xF5;
const REG_DATA: u8 = 0xF7;

const RESET_WORD: u8 = 0xB6;
const STATUS_MEASURING: u8 = 1 << 3;
const STATUS_IM_UPDATE: u8 = 1 << 0;
/// osrs_t = x1, osrs_p = x1, mode = forced.
const CTRL_MEAS_FORCED_X1: u8 = (0b001 << 5) | (0b001 << 2) | 0b01;
/// Standby 0.5 ms, IIR filter off, SPI 3-wire off.
const CONFIG: u8 = 0x00;
/// Value the chip reports for a skipped measurement.
const ADC_SKIPPED: i32 = 0x80000;

/// Section 3.8.1: 5.5 ms typical at x1/x1.
const MEASURE_TYPICAL: Duration = Duration::from_micros(5500);
/// Section 3.8.1 gives 6.4 ms maximum at x1/x1; this adds margin.
const MEASURE_LIMIT: Duration = Duration::from_millis(20);
/// Section 1: start-up time 2 ms.
const STARTUP: Duration = Duration::from_millis(2);
const POLL: Duration = Duration::from_millis(1);
/// Budget for one bus transfer.
const TRANSFER: Duration = Duration::from_millis(50);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Bus(BusError),
    /// The device at the address reported this chip ID instead of 0x58.
    WrongChip(u8),
    /// The calibration words are unusable (dig_T1 or dig_P1 is zero).
    BadCalibration,
    /// `read` was called before a successful `init`.
    NotInitialized,
    /// A conversion or NVM copy did not finish in time.
    NotReady,
    /// The chip reported a skipped measurement.
    NoData,
    /// The clock cannot represent the deadline.
    Clock,
}

impl From<BusError> for Error {
    fn from(e: BusError) -> Self {
        Error::Bus(e)
    }
}

/// Factory trimming parameters, registers 0x88 to 0x9F.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Calibration {
    pub t1: u16,
    pub t2: i16,
    pub t3: i16,
    pub p1: u16,
    pub p2: i16,
    pub p3: i16,
    pub p4: i16,
    pub p5: i16,
    pub p6: i16,
    pub p7: i16,
    pub p8: i16,
    pub p9: i16,
}

impl Calibration {
    pub fn from_bytes(b: &[u8; 24]) -> Result<Self, Error> {
        let u = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
        let s = |i: usize| i16::from_le_bytes([b[i], b[i + 1]]);
        let c = Calibration {
            t1: u(0),
            t2: s(2),
            t3: s(4),
            p1: u(6),
            p2: s(8),
            p3: s(10),
            p4: s(12),
            p5: s(14),
            p6: s(16),
            p7: s(18),
            p8: s(20),
            p9: s(22),
        };
        if c.t1 == 0 || c.p1 == 0 {
            Err(Error::BadCalibration)
        } else {
            Ok(c)
        }
    }

    /// Temperature in 0.01 degC, and `t_fine` for the pressure formula.
    ///
    /// The datasheet formula is written in 32 bits, but device-supplied
    /// readings and trimming values can overflow it, so it runs in 64 bits,
    /// where no `i32` input can overflow. `None` if a result does not fit
    /// in 32 bits, which no 20-bit reading can cause.
    pub fn temperature(&self, adc_t: i32) -> Option<(i32, i32)> {
        let adc_t = i64::from(adc_t);
        let t1 = i64::from(self.t1);
        let var1 = (((adc_t >> 3) - (t1 << 1)) * i64::from(self.t2)) >> 11;
        let d = (adc_t >> 4) - t1;
        let var2 = (((d * d) >> 12) * i64::from(self.t3)) >> 14;
        let t_fine = var1 + var2;
        let centi_c = i32::try_from((t_fine * 5 + 128) >> 8).ok()?;
        Some((centi_c, i32::try_from(t_fine).ok()?))
    }

    /// Pressure in Pa as unsigned Q24.8, or `None` where the datasheet
    /// formula would divide by zero or overflow 64 bits. Both need trimming
    /// values or readings far outside what a working chip reports.
    pub fn pressure(&self, adc_p: i32, t_fine: i32) -> Option<u32> {
        let var1 = i64::from(t_fine) - 128_000;
        let var2 = var1.checked_mul(var1)?.checked_mul(i64::from(self.p6))?;
        let var2 = var2.checked_add(var1.checked_mul(i64::from(self.p5))?.checked_mul(1 << 17)?)?;
        let var2 = var2.checked_add(i64::from(self.p4) << 35)?;
        let var1 = (var1.checked_mul(var1)?.checked_mul(i64::from(self.p3))? >> 8)
            .checked_add(var1.checked_mul(i64::from(self.p2))?.checked_mul(1 << 12)?)?;
        let var1 = (1_i64 << 47)
            .checked_add(var1)?
            .checked_mul(i64::from(self.p1))?
            >> 33;
        let p = (1_048_576 - i64::from(adc_p))
            .checked_mul(1 << 31)?
            .checked_sub(var2)?
            .checked_mul(3125)?
            .checked_div(var1)?;
        let var1 = i64::from(self.p9)
            .checked_mul(p >> 13)?
            .checked_mul(p >> 13)?
            >> 25;
        let var2 = i64::from(self.p8).checked_mul(p)? >> 19;
        let p =
            (p.checked_add(var1)?.checked_add(var2)? >> 8).checked_add(i64::from(self.p7) << 4)?;
        u32::try_from(p).ok()
    }
}

pub struct Bmp280<I> {
    i2c: I,
    addr: u8,
    sensor_id: u32,
    epoch: u32,
    seq: u32,
    cal: Option<Calibration>,
    announce_epoch: bool,
    initialized_before: bool,
}

impl<I: I2c> Bmp280<I> {
    /// No bus traffic until [`Bmp280::init`].
    pub fn new(i2c: I, addr: u8, sensor_id: u32, epoch: u32) -> Self {
        Bmp280 {
            i2c,
            addr,
            sensor_id,
            epoch,
            seq: 0,
            cal: None,
            announce_epoch: true,
            initialized_before: false,
        }
    }

    pub fn release(self) -> I {
        self.i2c
    }

    pub fn calibration(&self) -> Option<Calibration> {
        self.cal
    }

    /// Checks the chip ID, resets the chip, waits for the calibration copy
    /// and reads the trimming parameters.
    pub fn init<C: Clock>(&mut self, clock: &C) -> Result<(), Error> {
        self.cal = None;
        let id = self.read_reg(clock, REG_ID)?;
        if id != CHIP_ID {
            return Err(Error::WrongChip(id));
        }
        self.write_reg(clock, REG_RESET, RESET_WORD)?;
        clock.sleep_until(after(clock, STARTUP)?);
        let deadline = after(clock, MEASURE_LIMIT)?;
        while self.read_reg(clock, REG_STATUS)? & STATUS_IM_UPDATE != 0 {
            wait_or_timeout(clock, deadline)?;
        }
        let mut raw = [0u8; 24];
        self.i2c
            .transfer(self.addr, &[REG_CALIB], &mut raw, after(clock, TRANSFER)?)?;
        let cal = Calibration::from_bytes(&raw)?;
        self.write_reg(clock, REG_CONFIG, CONFIG)?;
        self.cal = Some(cal);
        // A second init is a restart: a new epoch, and the sequence restarts.
        if self.initialized_before {
            self.epoch = self.epoch.wrapping_add(1);
        }
        self.initialized_before = true;
        self.seq = 0;
        self.announce_epoch = true;
        Ok(())
    }

    /// One forced-mode conversion. Channel 0 is pressure in Pa with three
    /// decimals, channel 1 temperature in degC with two.
    pub fn read<C: Clock>(&mut self, clock: &C) -> Result<Sample, Error> {
        let cal = self.cal.ok_or(Error::NotInitialized)?;
        self.write_reg(clock, REG_CTRL_MEAS, CTRL_MEAS_FORCED_X1)?;
        let deadline = after(clock, MEASURE_LIMIT)?;
        clock.sleep_until(after(clock, MEASURE_TYPICAL)?);
        while self.read_reg(clock, REG_STATUS)? & STATUS_MEASURING != 0 {
            wait_or_timeout(clock, deadline)?;
        }
        let mut d = [0u8; 6];
        self.i2c
            .transfer(self.addr, &[REG_DATA], &mut d, after(clock, TRANSFER)?)?;
        let timestamp = clock.now();

        let adc_p = (i32::from(d[0]) << 12) | (i32::from(d[1]) << 4) | (i32::from(d[2]) >> 4);
        let adc_t = (i32::from(d[3]) << 12) | (i32::from(d[4]) << 4) | (i32::from(d[5]) >> 4);
        if adc_p == ADC_SKIPPED || adc_t == ADC_SKIPPED {
            return Err(Error::NoData);
        }
        let (centi_c, t_fine) = cal.temperature(adc_t).ok_or(Error::NoData)?;
        let p_q24_8 = cal.pressure(adc_p, t_fine).ok_or(Error::NoData)?;
        // Q24.8 Pa to mPa; the result is below 2^31 for any pressure the
        // chip can report (300 to 1100 hPa).
        let milli_pa = i32::try_from(i64::from(p_q24_8) * 1000 / 256).map_err(|_| Error::NoData)?;

        let mut ch = [Channel::default(); MAX_CHANNELS];
        ch[0] = Channel {
            raw: adc_p,
            value: milli_pa,
            unit: Unit::Pascal as u16,
            exp: -3,
            reserved: 0,
        };
        ch[1] = Channel {
            raw: adc_t,
            value: centi_c,
            unit: Unit::DegC as u16,
            exp: -2,
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
            kind: Kind::Pressure as u16,
            flags: flags.0,
            ts_source: TsSource::DriverRead as u8,
            nchannels: 2,
            ch,
            ..Sample::default()
        };
        self.announce_epoch = false;
        self.seq = self.seq.wrapping_add(1);
        Ok(sample)
    }

    fn read_reg<C: Clock>(&mut self, clock: &C, reg: u8) -> Result<u8, Error> {
        let mut v = [0u8; 1];
        self.i2c
            .transfer(self.addr, &[reg], &mut v, after(clock, TRANSFER)?)?;
        Ok(v[0])
    }

    fn write_reg<C: Clock>(&mut self, clock: &C, reg: u8, value: u8) -> Result<(), Error> {
        self.i2c
            .transfer(self.addr, &[reg, value], &mut [], after(clock, TRANSFER)?)?;
        Ok(())
    }
}

fn after<C: Clock>(clock: &C, d: Duration) -> Result<Instant, Error> {
    clock.now().checked_add(d).ok_or(Error::Clock)
}

fn wait_or_timeout<C: Clock>(clock: &C, deadline: Instant) -> Result<(), Error> {
    if clock.now() >= deadline {
        return Err(Error::NotReady);
    }
    clock.sleep_until(after(clock, POLL)?.min(deadline));
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use sensor_fake::{FakeClock, FakeDevice, FakeI2c, RegisterDevice};

    /// Calibration read from a SunFounder BMP280 module on 2026-09-30.
    const MODULE_CALIB: [u8; 24] = [
        0x90, 0x6d, 0x6e, 0x63, 0x32, 0x00, 0x4d, 0x90, 0xc0, 0xd5, 0xd0, 0x0b, 0x31, 0x1b, 0xc8,
        0xff, 0xf9, 0xff, 0x8c, 0x3c, 0xf8, 0xc6, 0x70, 0x17,
    ];
    /// Data registers 0xF7..0xFC read from the same module.
    const MODULE_DATA: [u8; 6] = [0x55, 0x16, 0x00, 0x86, 0x8c, 0x00];

    /// A BMP280 whose forced conversions finish at once and hold MODULE_DATA.
    fn chip() -> RegisterDevice {
        let mut dev = RegisterDevice::new(ADDR_SDO_LOW).with_hook(Box::new(|reg, v, regs| {
            if reg == REG_RESET && v == RESET_WORD {
                regs[usize::from(REG_CTRL_MEAS)] = 0;
            }
            if reg == REG_CTRL_MEAS && v & 0b11 == 0b01 {
                regs[usize::from(REG_DATA)..usize::from(REG_DATA) + 6]
                    .copy_from_slice(&MODULE_DATA);
            }
        }));
        dev.regs[usize::from(REG_ID)] = CHIP_ID;
        dev.regs[usize::from(REG_CALIB)..usize::from(REG_CALIB) + 24]
            .copy_from_slice(&MODULE_CALIB);
        dev
    }

    #[test]
    fn datasheet_example_compensates() {
        // BST-BMP280-DS001 section 3.12: 25.08 degC and 100653.27 Pa.
        let cal = Calibration {
            t1: 27504,
            t2: 26435,
            t3: -1000,
            p1: 36477,
            p2: -10685,
            p3: 3024,
            p4: 2855,
            p5: 140,
            p6: -7,
            p7: 15500,
            p8: -14600,
            p9: 6000,
        };
        let (t, t_fine) = cal.temperature(519_888).unwrap();
        assert_eq!(t, 2508);
        let p = cal.pressure(415_148, t_fine).unwrap();
        assert!((i64::from(p) * 100 / 256 - 10_065_327).abs() <= 3, "{p}");
    }

    /// The datasheet's 32-bit temperature formula, or `None` where it
    /// overflows.
    fn temperature_i32(c: &Calibration, adc_t: i32) -> Option<(i32, i32)> {
        let t1 = i32::from(c.t1);
        let var1 = ((adc_t >> 3) - (t1 << 1)).checked_mul(i32::from(c.t2))? >> 11;
        let d = (adc_t >> 4) - t1;
        let var2 = (d.checked_mul(d)? >> 12).checked_mul(i32::from(c.t3))? >> 14;
        let t_fine = var1.checked_add(var2)?;
        Some((t_fine.checked_mul(5)?.checked_add(128)? >> 8, t_fine))
    }

    #[test]
    fn temperature_matches_the_32_bit_formula_where_it_does_not_overflow() {
        let mut compared = 0;
        for t1 in (20_000..=35_000).step_by(1_500) {
            for t2 in (20_000..=30_000).step_by(2_500) {
                for t3 in (-2_000..=0).step_by(500) {
                    let c = Calibration {
                        t1,
                        t2,
                        t3,
                        p1: 1,
                        ..Calibration::default()
                    };
                    for adc_t in (0x6_0000..=0x9_0000).step_by(0x1_000) {
                        if let Some(want) = temperature_i32(&c, adc_t) {
                            assert_eq!(c.temperature(adc_t), Some(want), "{c:?} {adc_t}");
                            compared += 1;
                        }
                    }
                }
            }
        }
        assert!(compared > 10_000, "{compared}");
    }

    #[test]
    fn extreme_readings_and_trimming_do_not_overflow() {
        // Full-scale reading with t1 = 1 overflows the 32-bit formula.
        let c = Calibration {
            t1: 1,
            t2: i16::MAX,
            t3: i16::MAX,
            p1: 1,
            ..Calibration::default()
        };
        assert_eq!(temperature_i32(&c, 0xF_FFFF), None);
        // Expected values computed with arbitrary-precision integers.
        assert_eq!(c.temperature(0xF_FFFF), Some((81_914, 4_194_000)));

        // Every corner of the trimming space, at both ends of the reading
        // range and of `t_fine`: each call returns without panicking, and any
        // value it returns is the exact one.
        //
        // One-sided on purpose. The checks are per step, so an input whose
        // intermediate overflows 64 bits but whose result would fit returns
        // `None` where the 128-bit reference has a value; `None` is only
        // counted. Plain arithmetic passes the `Some` half of this test but
        // panics on the corners, which is the regression it guards.
        let ends16 = [i16::MIN, i16::MAX];
        let mut none = 0;
        for &t1 in &[1, u16::MAX] {
            for &t2 in &ends16 {
                for &t3 in &ends16 {
                    for &adc_t in &[0, 0xF_FFFF] {
                        let tc = Calibration {
                            t1,
                            t2,
                            t3,
                            p1: 1,
                            ..Calibration::default()
                        };
                        let (_, t_fine) = tc.temperature(adc_t).unwrap();
                        for t_fine in [t_fine, i32::MIN, i32::MAX] {
                            for bits in 0..1u32 << 9 {
                                let e = |i: u32| ends16[((bits >> i) & 1) as usize];
                                let c = Calibration {
                                    p1: if bits & 1 == 0 { 1 } else { u16::MAX },
                                    p2: e(1),
                                    p3: e(2),
                                    p4: e(3),
                                    p5: e(4),
                                    p6: e(5),
                                    p7: e(6),
                                    p8: e(7),
                                    p9: e(8),
                                    ..tc
                                };
                                for &adc_p in &[0, 0xF_FFFF] {
                                    match c.pressure(adc_p, t_fine) {
                                        Some(p) => assert_eq!(
                                            pressure_i128(&c, adc_p, t_fine),
                                            Some(p),
                                            "{c:?} {adc_p} {t_fine}"
                                        ),
                                        None => none += 1,
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        assert!(none > 0, "no corner exercised the overflow checks");
    }

    /// The pressure formula in 128 bits: exact wherever it returns a value.
    fn pressure_i128(c: &Calibration, adc_p: i32, t_fine: i32) -> Option<u32> {
        let v1 = i128::from(t_fine) - 128_000;
        let v2 = v1.checked_mul(v1)?.checked_mul(i128::from(c.p6))?
            + v1 * i128::from(c.p5) * (1 << 17)
            + i128::from(c.p4) * (1 << 35);
        let v1 = ((v1 * v1 * i128::from(c.p3)) >> 8) + v1 * i128::from(c.p2) * (1 << 12);
        let v1 = (((1_i128 << 47) + v1) * i128::from(c.p1)) >> 33;
        let p = (((1_048_576 - i128::from(adc_p)) << 31) - v2)
            .checked_mul(3125)?
            .checked_div(v1)?;
        let v1 = i128::from(c.p9)
            .checked_mul(p >> 13)?
            .checked_mul(p >> 13)?
            >> 25;
        let v2 = (i128::from(c.p8) * p) >> 19;
        u32::try_from(((p + v1 + v2) >> 8) + (i128::from(c.p7) << 4)).ok()
    }

    #[test]
    fn a_zero_divisor_is_not_a_pressure() {
        // p1 = 0 is refused by `from_bytes`, but the fields are public.
        let c = Calibration {
            t1: 27504,
            t2: 26435,
            p1: 0,
            ..Calibration::default()
        };
        let (_, t_fine) = c.temperature(519_888).unwrap();
        assert_eq!(c.pressure(415_148, t_fine), None);
    }

    #[test]
    fn module_reading_matches_the_hand_calculation() {
        let clock = FakeClock::new();
        let mut s = Bmp280::new(FakeI2c::with_device(chip()), ADDR_SDO_LOW, 5, 1);
        s.init(&clock).unwrap();
        let sample = s.read(&clock).unwrap();
        let ch = sample.channels().unwrap();
        assert_eq!(ch[0].value, 100_526_390); // 1005.26390 hPa
        assert_eq!(ch[1].value, 3106); // 31.06 degC
        assert_eq!((ch[0].raw, ch[1].raw), (348_512, 551_104));
        assert_eq!(sample.kind(), Some(Kind::Pressure));
    }

    #[test]
    fn init_resets_and_configures() {
        let clock = FakeClock::new();
        let mut s = Bmp280::new(FakeI2c::with_device(chip()), ADDR_SDO_LOW, 5, 1);
        s.init(&clock).unwrap();
        assert_eq!(
            s.release().register_writes(ADDR_SDO_LOW),
            [(REG_RESET, RESET_WORD), (REG_CONFIG, CONFIG)]
        );
        assert!(clock.now().as_nanos() >= STARTUP.as_nanos() as i64);
    }

    #[test]
    fn registers_and_bits_match_the_datasheet() {
        // Section 4.3: id 0xD0, reset 0xE0 with 0xB6, status 0xF3 (measuring
        // bit 3, im_update bit 0), ctrl_meas 0xF4, config 0xF5, data 0xF7.
        // ctrl_meas 0x25 is osrs_t x1, osrs_p x1, forced mode.
        let clock = FakeClock::new();
        let mut s = Bmp280::new(FakeI2c::with_device(chip()), ADDR_SDO_LOW, 5, 1);
        s.init(&clock).unwrap();
        s.read(&clock).unwrap();
        let i2c = s.release();
        assert_eq!(
            i2c.register_writes(0x76),
            [(0xE0, 0xB6), (0xF5, 0x00), (0xF4, 0x25)]
        );
        let reads: Vec<_> = i2c
            .log
            .iter()
            .filter(|t| t.read_len > 0)
            .map(|t| (t.write[0], t.read_len))
            .collect();
        assert_eq!(
            reads,
            [(0xD0, 1), (0xF3, 1), (0x88, 24), (0xF3, 1), (0xF7, 6)]
        );
    }

    /// The chip, with its status register busy for a number of reads after a
    /// reset (im_update, 0x01) or after a forced conversion starts
    /// (measuring, 0x08).
    struct Busy {
        inner: RegisterDevice,
        after_reset: usize,
        after_measure: usize,
        pending: usize,
        bits: u8,
    }

    impl FakeDevice for Busy {
        fn addr(&self) -> u8 {
            self.inner.addr
        }

        fn transfer(&mut self, w: &[u8], r: &mut [u8]) -> Result<(), BusError> {
            if w == [0xE0, 0xB6] {
                (self.pending, self.bits) = (self.after_reset, 0x01);
            }
            if w.len() == 2 && w[0] == 0xF4 && w[1] & 0b11 == 0b01 {
                (self.pending, self.bits) = (self.after_measure, 0x08);
            }
            if w == [0xF3] && !r.is_empty() {
                self.inner.regs[0xF3] = if self.pending > 0 {
                    self.pending -= 1;
                    self.bits
                } else {
                    0
                };
            }
            self.inner.transfer(w, r)
        }
    }

    fn busy(after_reset: usize, after_measure: usize) -> FakeI2c {
        FakeI2c::with_device(Busy {
            inner: chip(),
            after_reset,
            after_measure,
            pending: 0,
            bits: 0,
        })
    }

    fn status_reads(i2c: &FakeI2c) -> usize {
        i2c.log
            .iter()
            .filter(|t| t.write == [0xF3] && t.read_len == 1)
            .count()
    }

    #[test]
    fn init_waits_for_the_calibration_copy() {
        let clock = FakeClock::new();
        let mut s = Bmp280::new(busy(3, 0), ADDR_SDO_LOW, 5, 1);
        s.init(&clock).unwrap();
        assert_eq!(status_reads(&s.release()), 4);
    }

    #[test]
    fn read_waits_while_measuring() {
        let clock = FakeClock::new();
        let mut s = Bmp280::new(busy(0, 3), ADDR_SDO_LOW, 5, 1);
        s.init(&clock).unwrap();
        let before = clock.now();
        s.read(&clock).unwrap();
        let waited = clock.now().saturating_duration_since(before);
        assert!(waited >= MEASURE_TYPICAL + 3 * POLL, "{waited:?}");
        assert_eq!(status_reads(&s.release()), 1 + 4);
    }

    #[test]
    fn a_second_init_starts_a_new_epoch() {
        let clock = FakeClock::new();
        let mut s = Bmp280::new(FakeI2c::with_device(chip()), ADDR_SDO_LOW, 5, 7);
        s.init(&clock).unwrap();
        s.read(&clock).unwrap();
        s.read(&clock).unwrap();
        s.init(&clock).unwrap();
        let after = s.read(&clock).unwrap();
        assert_eq!((after.epoch, after.seq), (8, 0));
        assert!(after.flags().contains(Flags::NEW_EPOCH));
    }

    #[test]
    fn a_different_chip_is_refused() {
        let mut dev = chip();
        dev.regs[usize::from(REG_ID)] = 0x60; // BME280
        let mut s = Bmp280::new(FakeI2c::with_device(dev), ADDR_SDO_LOW, 5, 1);
        assert_eq!(s.init(&FakeClock::new()), Err(Error::WrongChip(0x60)));
        assert_eq!(s.read(&FakeClock::new()), Err(Error::NotInitialized));
    }

    #[test]
    fn a_missing_chip_is_a_nak() {
        let mut s = Bmp280::new(FakeI2c::with_device(chip()), ADDR_SDO_HIGH, 5, 1);
        assert_eq!(s.init(&FakeClock::new()), Err(Error::Bus(BusError::Nak)));
    }

    #[test]
    fn a_stuck_conversion_times_out() {
        let clock = FakeClock::new();
        let mut dev = chip();
        dev.regs[0xF3] = 0x08; // measuring, datasheet section 4.3.3
        let mut s = Bmp280::new(FakeI2c::with_device(dev), ADDR_SDO_LOW, 5, 1);
        s.init(&clock).unwrap();
        let start = clock.now();
        assert_eq!(s.read(&clock), Err(Error::NotReady));
        assert!(clock.now().saturating_duration_since(start) <= MEASURE_LIMIT + POLL);
    }

    #[test]
    fn a_skipped_measurement_is_not_data() {
        let clock = FakeClock::new();
        let dev = chip().with_hook(Box::new(|reg, _, regs| {
            if reg == REG_CTRL_MEAS {
                regs[0xF7..0xFD].copy_from_slice(&[0x80, 0, 0, 0x80, 0, 0]);
            }
        }));
        let mut s = Bmp280::new(FakeI2c::with_device(dev), ADDR_SDO_LOW, 5, 1);
        s.init(&clock).unwrap();
        assert_eq!(s.read(&clock), Err(Error::NoData));
    }

    #[test]
    fn samples_carry_sequence_and_epoch() {
        let clock = FakeClock::new();
        let mut s = Bmp280::new(FakeI2c::with_device(chip()), ADDR_SDO_LOW, 5, 7);
        s.init(&clock).unwrap();
        let a = s.read(&clock).unwrap();
        let b = s.read(&clock).unwrap();
        assert_eq!((a.seq, b.seq, a.epoch, a.sensor_id), (0, 1, 7, 5));
        assert!(a.flags().contains(Flags::NEW_EPOCH));
        assert!(!b.flags().contains(Flags::NEW_EPOCH));
        assert!(b.timestamp > a.timestamp);
    }

    #[test]
    fn a_bus_error_is_reported() {
        let clock = FakeClock::new();
        let mut i2c = FakeI2c::with_device(chip());
        i2c.fail_next = Some(BusError::Io);
        let mut s = Bmp280::new(i2c, ADDR_SDO_LOW, 5, 1);
        assert_eq!(s.init(&clock), Err(Error::Bus(BusError::Io)));
    }
}
