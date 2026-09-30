// SPDX-License-Identifier: BSD-3-Clause
//! Linux transport: the monotonic clock and I2C through `/dev/i2c-N`.
//!
//! This crate is platform code, so it is the one place that calls into libc.
//! Every `unsafe` block is a single system call on memory this code owns.

use std::fs::{File, OpenOptions};
use std::io;
use std::os::fd::AsRawFd;
use std::time::Duration;

use sensor_core::bus::{BusError, Clock, I2c, Instant};

/// `CLOCK_MONOTONIC`, in nanoseconds.
#[derive(Clone, Copy, Debug, Default)]
pub struct MonotonicClock;

impl Clock for MonotonicClock {
    fn now(&self) -> Instant {
        let mut ts = libc::timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        // SAFETY: `ts` is a valid, writable timespec for the duration of the call.
        let rc = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
        assert_eq!(rc, 0, "CLOCK_MONOTONIC is always available on Linux");
        // time_t and c_long are 32-bit on some targets, so keep the widening.
        #[allow(clippy::useless_conversion)]
        let ns = i64::from(ts.tv_sec) * 1_000_000_000 + i64::from(ts.tv_nsec);
        Instant::from_nanos(ns)
    }

    fn sleep_until(&self, t: Instant) {
        loop {
            let left = t.saturating_duration_since(self.now());
            if left.is_zero() {
                return;
            }
            std::thread::sleep(left.min(Duration::from_secs(1)));
        }
    }
}

// linux/i2c-dev.h and linux/i2c.h
const I2C_RDWR: libc::c_ulong = 0x0707;
const I2C_M_RD: u16 = 0x0001;

#[repr(C)]
struct I2cMsg {
    addr: u16,
    flags: u16,
    len: u16,
    buf: *mut u8,
}

#[repr(C)]
struct I2cRdwrData {
    msgs: *mut I2cMsg,
    nmsgs: u32,
}

/// One I2C adapter. A write followed by a read is issued as one `I2C_RDWR`
/// call, so the kernel joins them with a repeated start.
///
/// The deadline is checked before the transfer; the transfer itself is bounded
/// by the adapter's own kernel timeout.
pub struct LinuxI2c {
    dev: File,
    clock: MonotonicClock,
}

impl LinuxI2c {
    pub fn open(bus: u32) -> io::Result<Self> {
        let dev = OpenOptions::new()
            .read(true)
            .write(true)
            .open(format!("/dev/i2c-{bus}"))?;
        Ok(LinuxI2c {
            dev,
            clock: MonotonicClock,
        })
    }
}

impl I2c for LinuxI2c {
    fn transfer(
        &mut self,
        addr: u8,
        write: &[u8],
        read: &mut [u8],
        deadline: Instant,
    ) -> Result<(), BusError> {
        if addr > 0x7f || write.len() > usize::from(u16::MAX) || read.len() > usize::from(u16::MAX)
        {
            return Err(BusError::Invalid);
        }
        if write.is_empty() && read.is_empty() {
            return Ok(());
        }
        if self.clock.now() >= deadline {
            return Err(BusError::Timeout);
        }
        let mut msgs: [I2cMsg; 2] = [
            I2cMsg {
                addr: addr.into(),
                flags: 0,
                len: write.len() as u16,
                buf: write.as_ptr() as *mut u8,
            },
            I2cMsg {
                addr: addr.into(),
                flags: I2C_M_RD,
                len: read.len() as u16,
                buf: read.as_mut_ptr(),
            },
        ];
        let (first, count) = match (write.is_empty(), read.is_empty()) {
            (false, false) => (0, 2),
            (false, true) => (0, 1),
            (true, _) => (1, 1),
        };
        let mut data = I2cRdwrData {
            msgs: msgs[first..].as_mut_ptr(),
            nmsgs: count,
        };
        // SAFETY: `data` points at `count` initialised messages whose buffers
        // are live for the call. The kernel only reads write buffers (no
        // I2C_M_RD flag) and writes at most `len` bytes into the read buffer.
        let rc = unsafe { libc::ioctl(self.dev.as_raw_fd(), I2C_RDWR as _, &mut data) };
        if rc >= 0 {
            return Ok(());
        }
        Err(match io::Error::last_os_error().raw_os_error() {
            Some(libc::ENXIO) | Some(libc::EREMOTEIO) => BusError::Nak,
            Some(libc::ETIMEDOUT) => BusError::Timeout,
            Some(libc::EINVAL) => BusError::Invalid,
            _ => BusError::Io,
        })
    }
}

/// Command-line options shared by the read tools.
pub struct ReadArgs {
    pub bus: u32,
    pub addr: u8,
    pub count: Option<u64>,
    pub interval: Duration,
}

impl ReadArgs {
    /// Parses `--bus N --addr 0xNN --count N --interval-ms N`.
    pub fn parse(default_addr: u8, args: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut a = ReadArgs {
            bus: 1,
            addr: default_addr,
            count: None,
            interval: Duration::from_secs(1),
        };
        let mut it = args;
        while let Some(flag) = it.next() {
            let value = it.next().ok_or(format!("{flag} needs a value"))?;
            let bad = |_| format!("bad value for {flag}: {value}");
            match flag.as_str() {
                "--bus" => a.bus = value.parse().map_err(bad)?,
                "--addr" => {
                    a.addr = u8::from_str_radix(value.trim_start_matches("0x"), 16).map_err(bad)?
                }
                "--count" => a.count = Some(value.parse().map_err(bad)?),
                "--interval-ms" => a.interval = Duration::from_millis(value.parse().map_err(bad)?),
                _ => return Err(format!("unknown option {flag}")),
            }
        }
        Ok(a)
    }
}

/// Formats `value * 10^exp` without floating point.
pub fn decimal(value: i32, exp: i8) -> String {
    let v = i64::from(value);
    if exp >= 0 {
        return (v * 10_i64.pow(exp as u32)).to_string();
    }
    let scale = 10_i64.pow(u32::from(exp.unsigned_abs()));
    let sign = if v < 0 { "-" } else { "" };
    format!(
        "{sign}{}.{:0width$}",
        v.abs() / scale,
        v.abs() % scale,
        width = usize::from(exp.unsigned_abs())
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn options_parse_with_defaults() {
        let a = ReadArgs::parse(0x48, std::iter::empty()).unwrap();
        assert_eq!(
            (a.bus, a.addr, a.count, a.interval),
            (1, 0x48, None, Duration::from_secs(1))
        );
        let args = ["--addr", "0x76", "--count", "3", "--interval-ms", "250"].map(String::from);
        let a = ReadArgs::parse(0x48, args.into_iter()).unwrap();
        assert_eq!(
            (a.addr, a.count, a.interval),
            (0x76, Some(3), Duration::from_millis(250))
        );
        assert!(ReadArgs::parse(0x48, ["--addr".to_string()].into_iter()).is_err());
        assert!(ReadArgs::parse(0x48, ["--nope", "1"].map(String::from).into_iter()).is_err());
    }

    #[test]
    fn decimal_places_follow_the_exponent() {
        assert_eq!(decimal(100_526_390, -3), "100526.390");
        assert_eq!(decimal(3106, -2), "31.06");
        assert_eq!(decimal(-5, -2), "-0.05");
        assert_eq!(decimal(42, 0), "42");
        assert_eq!(decimal(7, 2), "700");
    }

    #[test]
    fn the_clock_moves_forward() {
        let c = MonotonicClock;
        let a = c.now();
        c.sleep_until(a.checked_add(Duration::from_millis(2)).unwrap());
        assert!(c.now().saturating_duration_since(a) >= Duration::from_millis(2));
    }
}
