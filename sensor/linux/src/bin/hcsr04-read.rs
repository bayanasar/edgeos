// SPDX-License-Identifier: BSD-3-Clause
//! Reads an HC-SR04 on two lines of a Linux GPIO chip at a fixed interval.
//!
//! Usage: hcsr04-read [--chip PATH] [--trig N] [--echo N] [--temp-c N]
//!                    [--count N] [--interval-ms N]
//! `--trig` and `--echo` are line offsets on the chip (on a Raspberry Pi,
//! the BCM GPIO numbers). `--temp-c` is the air temperature in whole degC,
//! for the speed of sound; the default assumes 20 degC.

use std::process::ExitCode;
use std::time::Duration;

use sensor_core::bus::Clock;
use sensor_core::sample::Flags;
use sensor_hcsr04::HcSr04;
use sensor_linux::{LinuxGpio, MonotonicClock, decimal};

const USAGE: &str = "usage: hcsr04-read [--chip PATH] [--trig N] [--echo N] [--temp-c N] \
                     [--count N] [--interval-ms N]";

/// Not yet assigned a module number from the kit's manual.
const SENSOR_ID: u32 = 0;

struct Args {
    chip: String,
    trig: u32,
    echo: u32,
    temp_c: Option<i32>,
    count: Option<u64>,
    interval: Duration,
}

fn parse(args: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut a = Args {
        chip: "/dev/gpiochip0".into(),
        trig: 23,
        echo: 24,
        temp_c: None,
        count: None,
        interval: Duration::from_millis(200),
    };
    let mut it = args;
    while let Some(flag) = it.next() {
        let value = it.next().ok_or(format!("{flag} needs a value"))?;
        let bad = |_| format!("bad value for {flag}: {value}");
        match flag.as_str() {
            "--chip" => a.chip = value.clone(),
            "--trig" => a.trig = value.parse().map_err(bad)?,
            "--echo" => a.echo = value.parse().map_err(bad)?,
            "--temp-c" => a.temp_c = Some(value.parse().map_err(bad)?),
            "--count" => a.count = Some(value.parse().map_err(bad)?),
            "--interval-ms" => a.interval = Duration::from_millis(value.parse().map_err(bad)?),
            _ => return Err(format!("unknown option {flag}")),
        }
    }
    Ok(a)
}

fn main() -> ExitCode {
    let args = match parse(std::env::args().skip(1)) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}\n{USAGE}");
            return ExitCode::from(2);
        }
    };
    let gpio = match LinuxGpio::open(&args.chip) {
        Ok(g) => g,
        Err(e) => {
            eprintln!("cannot open {}: {e}", args.chip);
            return ExitCode::FAILURE;
        }
    };
    let clock = MonotonicClock;
    let mut sensor = HcSr04::new(gpio, args.trig, args.echo, SENSOR_ID, 1);
    if let Some(t) = args.temp_c {
        sensor.set_air_temperature(t * 100);
    }
    if let Err(e) = sensor.init() {
        eprintln!("cannot configure the lines: {e:?}");
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
                let note = if s.flags().contains(Flags::SATURATED) {
                    "  (outside 2 cm to 4 m)"
                } else {
                    ""
                };
                println!(
                    "seq={:<4} t={:>7} ms  distance={} m  echo={} us{note}",
                    s.seq,
                    (s.timestamp - start.as_nanos()) / 1_000_000,
                    decimal(ch[0].value, ch[0].exp),
                    ch[0].raw / 1_000,
                );
            }
            Err(e) => eprintln!("read failed: {e:?}"),
        }
        n += 1;
        next = next.checked_add(args.interval).expect("clock range");
    }
    ExitCode::SUCCESS
}
