// SPDX-License-Identifier: BSD-3-Clause
//! Reads the four inputs of a PCF8591 on a Linux I2C bus at a fixed interval.
//!
//! Usage: pcf8591-read [--bus N] [--addr 0x48] [--count N] [--interval-ms N]
//! The reference is taken as 3.3 V, the Raspberry Pi supply.

use std::process::ExitCode;

use sensor_core::bus::Clock;
use sensor_linux::{LinuxI2c, MonotonicClock, ReadArgs, decimal};
use sensor_pcf8591::{BASE_ADDR, Pcf8591};

const USAGE: &str = "usage: pcf8591-read [--bus N] [--addr 0x48] [--count N] [--interval-ms N]";
const VREF_MV: u16 = 3300;

fn main() -> ExitCode {
    let args = match ReadArgs::parse(BASE_ADDR, std::env::args().skip(1)) {
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
    let mut adc = Pcf8591::new(i2c, args.addr, VREF_MV, 33, 1);
    let start = clock.now();
    let mut next = start;
    let mut n = 0u64;
    while args.count.is_none_or(|c| n < c) {
        clock.sleep_until(next);
        match adc.read(&clock) {
            Ok(s) => {
                let line: Vec<String> = s
                    .channels()
                    .unwrap_or(&[])
                    .iter()
                    .enumerate()
                    .map(|(i, c)| format!("AIN{i}={} V ({:>3})", decimal(c.value, c.exp), c.raw))
                    .collect();
                println!(
                    "seq={:<4} t={:>7} ms  {}",
                    s.seq,
                    (s.timestamp - start.as_nanos()) / 1_000_000,
                    line.join("  ")
                );
            }
            Err(e) => eprintln!("read failed: {e:?}"),
        }
        n += 1;
        next = next.checked_add(args.interval).expect("clock range");
    }
    ExitCode::SUCCESS
}
