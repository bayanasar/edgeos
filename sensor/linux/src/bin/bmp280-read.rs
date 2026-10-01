// SPDX-License-Identifier: BSD-3-Clause
//! Reads a BMP280 on a Linux I2C bus at a fixed interval and prints each sample.
//!
//! Usage: bmp280-read [--bus N] [--addr 0x76] [--count N] [--interval-ms N]

use std::process::ExitCode;

use sensor_bmp280::{ADDR_SDO_LOW, Bmp280};
use sensor_core::bus::Clock;
use sensor_linux::{LinuxI2c, MonotonicClock, ReadArgs, decimal};

const USAGE: &str = "usage: bmp280-read [--bus N] [--addr 0x76] [--count N] [--interval-ms N]";

fn main() -> ExitCode {
    let args = match ReadArgs::parse(ADDR_SDO_LOW, std::env::args().skip(1)) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}\n{USAGE}");
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
                println!(
                    "seq={:<4} t={:>7} ms  pressure={} Pa  temperature={} degC",
                    s.seq,
                    (s.timestamp - start.as_nanos()) / 1_000_000,
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
