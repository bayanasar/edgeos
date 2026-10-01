// SPDX-License-Identifier: BSD-3-Clause
//! Reads a DS18B20 on a Linux w1 bus master at a fixed interval.
//!
//! Usage: ds18b20-read [--bus N] [--count N] [--interval-ms N]
//! `--bus` is the w1 master number (`w1_bus_masterN`); the bus must hold one
//! device, since the driver addresses it with Skip ROM.

use std::process::ExitCode;

use sensor_core::bus::Clock;
use sensor_ds18b20::Ds18b20;
use sensor_linux::{LinuxW1, MonotonicClock, ReadArgs, decimal};

const USAGE: &str = "usage: ds18b20-read [--bus N] [--count N] [--interval-ms N]";

fn main() -> ExitCode {
    let args = match ReadArgs::parse(0, std::env::args().skip(1)) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}\n{USAGE}");
            return ExitCode::from(2);
        }
    };
    let w1 = match LinuxW1::open(args.bus) {
        Ok(w) => w,
        Err(e) => {
            eprintln!("cannot open the w1 netlink socket: {e}");
            return ExitCode::FAILURE;
        }
    };
    let clock = MonotonicClock;
    let mut sensor = Ds18b20::new(w1, 23, 1);
    let start = clock.now();
    let mut next = start;
    let mut n = 0u64;
    while args.count.is_none_or(|c| n < c) {
        clock.sleep_until(next);
        match sensor.read(&clock) {
            Ok(s) => {
                let ch = s.channels().unwrap_or(&[]);
                println!(
                    "seq={:<4} t={:>7} ms  temperature={} degC  (raw {})",
                    s.seq,
                    (s.timestamp - start.as_nanos()) / 1_000_000,
                    decimal(ch[0].value, ch[0].exp),
                    ch[0].raw,
                );
            }
            Err(e) => eprintln!("read failed: {e:?}"),
        }
        n += 1;
        next = next.checked_add(args.interval).expect("clock range");
    }
    ExitCode::SUCCESS
}
