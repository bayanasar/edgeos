// SPDX-License-Identifier: BSD-3-Clause
//! Reads a BMP280 on a Linux I2C bus at a fixed interval and prints each sample.
//!
//! Usage: bmp280-read [--bus N] [--addr 0x76] [--count N] [--interval-ms N]

use std::process::ExitCode;
use std::time::Duration;

use sensor_bmp280::{ADDR_SDO_LOW, Bmp280};
use sensor_core::bus::Clock;
use sensor_linux::{LinuxI2c, MonotonicClock, decimal};

struct Args {
    bus: u32,
    addr: u8,
    count: Option<u64>,
    interval: Duration,
}

fn parse() -> Result<Args, String> {
    let mut a = Args {
        bus: 1,
        addr: ADDR_SDO_LOW,
        count: None,
        interval: Duration::from_secs(1),
    };
    let mut it = std::env::args().skip(1);
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

fn main() -> ExitCode {
    let args = match parse() {
        Ok(a) => a,
        Err(e) => {
            eprintln!(
                "{e}\nusage: bmp280-read [--bus N] [--addr 0x76] [--count N] [--interval-ms N]"
            );
            return ExitCode::from(2);
        }
    };
    let i2c = match LinuxI2c::open(args.bus) {
        Ok(i) => i,
        Err(e) => {
            eprintln!("cannot open /dev/i2c-{}: {e}", args.bus);
            return ExitCode::FAILURE;
        }
    };
    let clock = MonotonicClock;
    let mut sensor = Bmp280::new(i2c, args.addr, 5, 1);
    if let Err(e) = sensor.init(&clock) {
        eprintln!(
            "BMP280 at 0x{:02x} on bus {}: init failed: {e:?}",
            args.addr, args.bus
        );
        return ExitCode::FAILURE;
    }
    let start = clock.now();
    let mut next = start;
    let mut n = 0u64;
    while args.count.is_none_or(|c| n < c) {
        clock.sleep_until(next);
        match sensor.read(&clock) {
            Ok(s) => {
                let ch = s.channels().unwrap_or(&[]);
                let ms = (s.timestamp - start.as_nanos()) / 1_000_000;
                println!(
                    "seq={:<4} t={:>7} ms  pressure={} Pa  temperature={} degC",
                    s.seq,
                    ms,
                    decimal(ch[0].value, ch[0].exp),
                    decimal(ch[1].value, ch[1].exp),
                );
            }
            Err(e) => eprintln!("read failed: {e:?}"),
        }
        n += 1;
        next = next.checked_add(args.interval).expect("clock range");
    }
    ExitCode::SUCCESS
}
