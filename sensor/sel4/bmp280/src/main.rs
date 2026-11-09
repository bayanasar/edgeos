// SPDX-License-Identifier: BSD-3-Clause
//! The BMP280 driver in a protection domain with no device access at all.
//! It reaches the sensor through the I2C controller's domain, sleeps through
//! the timer's domain, and hands each sample to its consumer by copy, in
//! message registers.

#![no_std]
#![no_main]

use core::cell::Cell;

use pd_common::{
    BMP280_FROM_CONSUMER, BMP280_TO_I2C, BMP280_TO_TIMER, CounterClock, ERR_REQUEST, I2cClient,
    OP_I2C_REPORT, OP_READ_SAMPLE, OP_REPORT, Timeout, TimerClient, sample_status, wait_input,
};
use sel4_microkit::{
    Channel, ChannelSet, Handler, Infallible, MessageInfo, debug_println, protection_domain,
    with_msg_regs_mut,
};
use sensor_bmp280::{ADDR_SDO_LOW, Bmp280};
use sensor_core::bus::{Clock, Instant};
use sensor_wire::{SAMPLE_WORDS, sample_to_words};

/// The sensor's ID in its samples.
const SENSOR_ID: u32 = 1;

/// The counter clock, with sleeps that block instead of spinning: the 2 ms
/// start-up after a reset, the 5.5 ms of each conversion and the 1 ms
/// between status polls.
///
/// A sleep asks the timer's domain for a signal at its end and blocks with
/// [`wait_input`] inside the protected procedure that serves the consumer's
/// request, so lower priorities run in the meantime. That is sound only
/// because the domain has one client, the consumer, blocked in the call
/// being served (see [`wait_input`]).
struct TimerClock {
    clock: CounterClock,
    timer: TimerClient,
    sleeps: Cell<Sleeps>,
}

/// How each sleep ended, so a run shows which path it took. A sleep whose
/// end has already passed when it starts is not counted.
#[derive(Default, Clone, Copy)]
struct Sleeps {
    /// Woken by the timer's signal at or after its end, by the counter
    /// clock.
    timer: u32,
    /// Woken by the timer's signal before its end, by the counter clock. The
    /// clock overrules it, and the sleep asks again.
    early: u32,
    /// Spun to the end, because the timer could not help: its clock had
    /// passed the end, or it does not count.
    spun: u32,
}

impl Clock for TimerClock {
    fn now(&self) -> Instant {
        self.clock.now()
    }

    fn sleep_until(&self, t: Instant) {
        if self.clock.now() >= t {
            return;
        }
        let mut s = self.sleeps.get();
        loop {
            if self.timer.set(t) != Timeout::Armed {
                self.clock.sleep_until(t);
                s.spun += 1;
                break;
            }
            while wait_input() & (1 << BMP280_TO_TIMER.index()) == 0 {}
            if self.clock.now() >= t {
                s.timer += 1;
                break;
            }
            s.early += 1;
        }
        self.sleeps.set(s);
    }
}

#[protection_domain]
fn init() -> HandlerImpl {
    debug_println!(
        "bmp280: expects a BMP280 at {:#04x}, no device mapped",
        ADDR_SDO_LOW
    );
    HandlerImpl {
        sensor: Bmp280::new(I2cClient::new(BMP280_TO_I2C), ADDR_SDO_LOW, SENSOR_ID, 1),
        clock: TimerClock {
            clock: CounterClock::new(),
            timer: TimerClient::new(BMP280_TO_TIMER),
            sleeps: Cell::new(Sleeps::default()),
        },
        ready: false,
    }
}

struct HandlerImpl {
    sensor: Bmp280<I2cClient>,
    clock: TimerClock,
    ready: bool,
}

impl HandlerImpl {
    /// Prints how the sleeps so far have ended.
    fn report(&self) {
        let s = self.clock.sleeps.get();
        debug_println!(
            "BMP280_SLEEPS: timer={} early={} spun={}",
            s.timer,
            s.early,
            s.spun
        );
    }
}

impl Handler for HandlerImpl {
    type Error = Infallible;

    fn protected(
        &mut self,
        channel: Channel,
        msg_info: MessageInfo,
    ) -> Result<MessageInfo, Self::Error> {
        if channel != BMP280_FROM_CONSUMER {
            return Ok(MessageInfo::new(ERR_REQUEST, 0));
        }
        match msg_info.label() {
            OP_READ_SAMPLE => {}
            OP_REPORT => {
                self.report();
                let reply = BMP280_TO_I2C.pp_call(MessageInfo::new(OP_I2C_REPORT, 0));
                return Ok(MessageInfo::new(reply.label(), 0));
            }
            _ => return Ok(MessageInfo::new(ERR_REQUEST, 0)),
        }
        // Initialise on demand, so a sensor connected later is found by the
        // next request.
        if !self.ready {
            if let Err(e) = self.sensor.init(&self.clock) {
                return Ok(MessageInfo::new(sample_status(Some(e)), 0));
            }
            self.ready = true;
        }
        match self.sensor.read(&self.clock) {
            Ok(sample) => {
                let words = sample_to_words(&sample);
                with_msg_regs_mut(|mrs| mrs[..SAMPLE_WORDS].copy_from_slice(&words));
                Ok(MessageInfo::new(0, SAMPLE_WORDS))
            }
            Err(e) => {
                self.ready = false;
                Ok(MessageInfo::new(sample_status(Some(e)), 0))
            }
        }
    }

    /// Between requests a signal can only be a stray: every sleep waits for
    /// the signal of the deadline it set. Nothing to do.
    fn notified(&mut self, _channels: ChannelSet) -> Result<(), Self::Error> {
        Ok(())
    }
}
