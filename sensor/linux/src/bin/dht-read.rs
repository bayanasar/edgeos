// SPDX-License-Identifier: BSD-3-Clause
//! Reads a DHT11 or DHT22 on one line of a Linux GPIO chip.
//!
//! Usage: dht-read --model dht11|dht22 [--chip PATH] [--line N] [--count N]
//!                 [--verbose]
//! `--line` is the line offset on the chip (on a Raspberry Pi, the BCM GPIO
//! number). The model must be named: the two answer alike. Reads are 2 s
//! apart, as both manuals ask. `--verbose` prints what each frame looked
//! like; a failed read always does. A tally is printed at the end.

use std::process::ExitCode;

use sensor_core::bus::Clock;
use sensor_core::sample::Flags;
use sensor_dht::{Dht, FrameInfo, Model};
use sensor_linux::{LinuxGpio, MonotonicClock, decimal};

const USAGE: &str =
    "usage: dht-read --model dht11|dht22 [--chip PATH] [--line N] [--count N] [--verbose]";

/// The module's lesson number in the kit's manual; 0 until the module on
/// the bench is identified.
const SENSOR_ID: u32 = 0;

struct Args {
    chip: String,
    line: u32,
    model: Model,
    count: Option<u64>,
    verbose: bool,
}

fn parse(args: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut chip = "/dev/gpiochip0".to_string();
    let mut line = 25;
    let mut model = None;
    let mut count = None;
    let mut verbose = false;
    let mut it = args;
    while let Some(flag) = it.next() {
        if flag == "--verbose" {
            verbose = true;
            continue;
        }
        let value = it.next().ok_or(format!("{flag} needs a value"))?;
        let bad = || format!("bad value for {flag}: {value}");
        match flag.as_str() {
            "--chip" => chip = value.clone(),
            "--line" => line = value.parse().map_err(|_| bad())?,
            "--model" => {
                model = Some(match value.as_str() {
                    "dht11" => Model::Dht11,
                    "dht22" => Model::Dht22,
                    _ => return Err(bad()),
                })
            }
            "--count" => count = Some(value.parse().map_err(|_| bad())?),
            _ => return Err(format!("unknown option {flag}")),
        }
    }
    Ok(Args {
        chip,
        line,
        model: model.ok_or("--model is required")?,
        count,
        verbose,
    })
}

fn frame(f: &FrameInfo) -> String {
    let us = |ns: i64| ns as f64 / 1_000.0;
    format!(
        "edges={} first_edge={:.1} us  zero={:.1}..{:.1} us  one={:.1}..{:.1} us",
        f.edges,
        us(f.first_edge_ns),
        us(f.zero_ns.0),
        us(f.zero_ns.1),
        us(f.one_ns.0),
        us(f.one_ns.1),
    )
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
    let mut sensor = Dht::new(gpio, args.line, args.model, SENSOR_ID, 1);
    if let Err(e) = sensor.init() {
        eprintln!("cannot configure the line: {e:?}");
        return ExitCode::FAILURE;
    }
    let start = clock.now();
    let (mut ok, mut failed) = (0u64, 0u64);
    while args.count.is_none_or(|c| ok + failed < c) {
        match sensor.read(&clock) {
            Ok(s) => {
                ok += 1;
                let ch = s.channels().unwrap_or(&[]);
                let mut notes = String::new();
                if s.flags().contains(Flags::STALE) {
                    notes.push_str("  (previous measurement)");
                }
                if s.flags().contains(Flags::SATURATED) {
                    notes.push_str("  (outside the measuring range)");
                }
                println!(
                    "seq={:<4} t={:>7} ms  humidity={} %RH  temperature={} degC{notes}",
                    s.seq,
                    (s.timestamp - start.as_nanos()) / 1_000_000,
                    decimal(ch[0].value, ch[0].exp),
                    decimal(ch[1].value, ch[1].exp),
                );
                if args.verbose {
                    println!("          {}", frame(&sensor.last_frame()));
                }
            }
            Err(e) => {
                failed += 1;
                eprintln!("read failed: {e:?}  {}", frame(&sensor.last_frame()));
            }
        }
    }
    println!("ok={ok} failed={failed}");
    ExitCode::SUCCESS
}
