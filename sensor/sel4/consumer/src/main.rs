// SPDX-License-Identifier: BSD-3-Clause
//! Asks the BMP280 domain for samples and reports each one, then a summary
//! line a test harness can match.

#![no_std]
#![no_main]

use core::time::Duration;

use pd_common::{CONSUMER_TO_BMP280, CounterClock, OP_READ_SAMPLE};
use sel4_microkit::{MessageInfo, NullHandler, debug_println, protection_domain, with_msg_regs};
use sensor_core::bus::Clock;
use sensor_wire::{SAMPLE_WORDS, sample_from_words};

const REQUESTS: u32 = 5;
const INTERVAL: Duration = Duration::from_millis(500);
/// Bus error codes as [`pd_common::sample_status`] reports them.
const NAK: u64 = 0x100 + 2;
const TIMEOUT: u64 = 0x100 + 1;

#[protection_domain]
fn init() -> NullHandler {
    let clock = CounterClock::new();
    let (mut samples, mut nak, mut timeout, mut other) = (0, 0, 0, 0);
    for i in 0..REQUESTS {
        let start = clock.now();
        let reply = CONSUMER_TO_BMP280.pp_call(MessageInfo::new(OP_READ_SAMPLE, 0));
        let took_us = clock.now().saturating_duration_since(start).as_micros();
        match reply.label() {
            0 if reply.count() == SAMPLE_WORDS => {
                let sample = with_msg_regs(|mrs| sample_from_words(&mrs[..SAMPLE_WORDS]));
                match sample.as_ref().and_then(|s| s.channels().map(|c| (s, c))) {
                    Some((s, ch)) if ch.len() == 2 => {
                        samples += 1;
                        debug_println!(
                            "consumer: request {i}: seq {} pressure {}e{} Pa temperature {}e{} C ({took_us} us)",
                            s.seq,
                            ch[0].value,
                            ch[0].exp,
                            ch[1].value,
                            ch[1].exp
                        );
                    }
                    _ => {
                        other += 1;
                        debug_println!("consumer: request {i}: malformed sample");
                    }
                }
            }
            label => {
                match label {
                    NAK => nak += 1,
                    TIMEOUT => timeout += 1,
                    _ => other += 1,
                }
                debug_println!("consumer: request {i}: error {label:#x} ({took_us} us)");
            }
        }
        if let Some(t) = clock.now().checked_add(INTERVAL) {
            clock.sleep_until(t);
        }
    }
    debug_println!(
        "SENSOR_RESULT: samples={samples} nak={nak} timeout={timeout} other={other} of {REQUESTS}"
    );
    NullHandler::new()
}
