// SPDX-License-Identifier: BSD-3-Clause
//! Prints the detents and switch presses of a rotary encoder module on
//! three lines of a Linux GPIO chip.
//!
//! Usage: encoder-read [--chip PATH] [--clk N] [--dt N] [--sw N|none]
//!                     [--steps 1|2|4] [--seconds N]
//! Line numbers are offsets on the chip (on a Raspberry Pi, the BCM GPIO
//! numbers). `--steps` is quadrature steps per detent; `--seconds` stops
//! after that long, otherwise it runs until interrupted. Start it with the
//! shaft at rest.

use std::process::ExitCode;
use std::time::Duration;

use sensor_core::bus::Clock;
use sensor_core::sample::{EventType, Flags};
use sensor_encoder::{Encoder, Pins, Steps};
use sensor_linux::{LinuxGpio, MonotonicClock};

const USAGE: &str = "usage: encoder-read [--chip PATH] [--clk N] [--dt N] [--sw N|none] \
                     [--steps 1|2|4] [--seconds N]";

/// The module's lesson number in the kit's manual; 0 until the module on
/// the bench is identified.
const SENSOR_ID: u32 = 0;

struct Args {
    chip: String,
    pins: Pins,
    steps: Steps,
    run_for: Option<Duration>,
}

fn parse(args: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut a = Args {
        chip: "/dev/gpiochip0".into(),
        pins: Pins {
            clk: 17,
            dt: 27,
            sw: Some(22),
        },
        steps: Steps::Four,
        run_for: None,
    };
    let mut it = args;
    while let Some(flag) = it.next() {
        let value = it.next().ok_or(format!("{flag} needs a value"))?;
        let bad = || format!("bad value for {flag}: {value}");
        match flag.as_str() {
            "--chip" => a.chip = value.clone(),
            "--clk" => a.pins.clk = value.parse().map_err(|_| bad())?,
            "--dt" => a.pins.dt = value.parse().map_err(|_| bad())?,
            "--sw" if value == "none" => a.pins.sw = None,
            "--sw" => a.pins.sw = Some(value.parse().map_err(|_| bad())?),
            "--steps" => {
                a.steps = match value.as_str() {
                    "1" => Steps::One,
                    "2" => Steps::Two,
                    "4" => Steps::Four,
                    _ => return Err(bad()),
                }
            }
            "--seconds" => a.run_for = Some(Duration::from_secs(value.parse().map_err(|_| bad())?)),
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
    let mut enc = Encoder::new(gpio, args.pins, args.steps, SENSOR_ID, 1);
    if let Err(e) = enc.init() {
        eprintln!("cannot configure the lines: {e:?}");
        return ExitCode::FAILURE;
    }
    let start = clock.now();
    let end = args.run_for.map(|d| start.checked_add(d).expect("clock range"));
    let mut position = 0i64;
    loop {
        // Wake at least once a second, so a run with no end can be stopped.
        let tick = clock
            .now()
            .checked_add(Duration::from_secs(1))
            .expect("clock range");
        let deadline = end.map_or(tick, |e| e.min(tick));
        match enc.next_event(&clock, deadline) {
            Ok(Some(e)) => {
                let t = (e.timestamp - start.as_nanos()) / 1_000_000;
                let gap = if e.flags().contains(Flags::SEQ_GAP) {
                    "  (edges were lost before this)"
                } else {
                    ""
                };
                match e.event_type() {
                    Some(EventType::Delta) => {
                        position += i64::from(e.value);
                        let dir = if e.value > 0 { "clockwise" } else { "counter-clockwise" };
                        println!(
                            "seq={:<4} t={t:>7} ms  turn {dir:<17} position={position}{gap}",
                            e.seq
                        );
                    }
                    Some(EventType::State) => {
                        let what = if e.value != 0 { "pressed" } else { "released" };
                        println!("seq={:<4} t={t:>7} ms  switch {what}{gap}", e.seq);
                    }
                    _ => println!("seq={:<4} t={t:>7} ms  unexpected event {e:?}", e.seq),
                }
            }
            Ok(None) => {
                if end.is_some_and(|e| clock.now() >= e) {
                    break;
                }
            }
            Err(e) => {
                eprintln!("read failed: {e:?}");
                return ExitCode::FAILURE;
            }
        }
    }
    println!("position={position} rejected_edges={}", enc.rejected());
    ExitCode::SUCCESS
}
