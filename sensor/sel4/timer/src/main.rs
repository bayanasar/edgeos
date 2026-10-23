// SPDX-License-Identifier: BSD-3-Clause
//! A general-purpose timer in a protection domain of its own. It holds the
//! timer's register page and its interrupt, nothing else. Its one client asks
//! by protected procedure call for a signal at a deadline on the shared
//! counter clock, and the domain signals it on the same channel when the
//! timer's compare matches.

#![no_std]
#![no_main]

use core::ptr::NonNull;
use core::time::Duration;

use pd_common::{
    CounterClock, ERR_REQUEST, OP_TIMEOUT_CANCEL, OP_TIMEOUT_SET, TIMEOUT_PASSED, TIMER_FROM_I2C,
    TIMER_IRQ, TIMER_STOPPED,
};
use sel4_microkit::{
    Channel, ChannelSet, Handler, Infallible, MessageInfo, debug_println, memory_region_symbol,
    protection_domain, with_msg_regs,
};
use sensor_core::bus::{Clock, Instant};
use sensor_imx_gpt::{Arm, CNT, ImxGpt, Regs};

/// How long the timer's rate is measured for at start: 1 ms, which spins.
const MEASURE: Duration = Duration::from_millis(1);

/// The register page, mapped uncached.
struct Mmio(NonNull<u8>);

impl Regs for Mmio {
    fn read(&mut self, offset: usize) -> u32 {
        assert!(offset <= CNT && offset % 4 == 0);
        // SAFETY: the page is mapped for this domain alone and the offset is
        // one of the timer's registers.
        unsafe { self.0.as_ptr().add(offset).cast::<u32>().read_volatile() }
    }

    fn write(&mut self, offset: usize, value: u32) {
        assert!(offset <= CNT && offset % 4 == 0);
        // SAFETY: as above.
        unsafe { self.0.as_ptr().add(offset).cast::<u32>().write_volatile(value) }
    }
}

#[protection_domain]
fn init() -> HandlerImpl {
    let clock = CounterClock::new();
    let mut gpt = ImxGpt::start(Mmio(memory_region_symbol!(gpt_regs: *mut u8)));
    let counting = match gpt.calibrate(&clock, MEASURE) {
        Some(hz) => {
            debug_println!("timer: counts at {} Hz", hz);
            true
        }
        None => {
            debug_println!("timer: does not count; every deadline is refused");
            false
        }
    };
    HandlerImpl {
        gpt,
        clock,
        counting,
    }
}

struct HandlerImpl {
    gpt: ImxGpt<Mmio>,
    clock: CounterClock,
    counting: bool,
}

impl HandlerImpl {
    fn set(&mut self, deadline: Instant) -> u64 {
        if !self.counting {
            return TIMER_STOPPED;
        }
        let now = self.clock.now();
        if deadline <= now {
            self.gpt.disarm();
            return TIMEOUT_PASSED;
        }
        let ticks = self.gpt.ticks_for(deadline.saturating_duration_since(now));
        match self.gpt.arm(ticks) {
            Arm::Armed => 0,
            Arm::Passed => TIMEOUT_PASSED,
        }
    }
}

impl Handler for HandlerImpl {
    type Error = Infallible;

    fn protected(
        &mut self,
        channel: Channel,
        msg_info: MessageInfo,
    ) -> Result<MessageInfo, Self::Error> {
        if channel != TIMER_FROM_I2C {
            return Ok(MessageInfo::new(ERR_REQUEST, 0));
        }
        let label = match (msg_info.label(), msg_info.count()) {
            (OP_TIMEOUT_SET, 1) => {
                let ns = with_msg_regs(|mrs| mrs[0]) as i64;
                self.set(Instant::from_nanos(ns))
            }
            (OP_TIMEOUT_CANCEL, 0) => {
                self.gpt.disarm();
                0
            }
            _ => ERR_REQUEST,
        };
        Ok(MessageInfo::new(label, 0))
    }

    fn notified(&mut self, channels: ChannelSet) -> Result<(), Self::Error> {
        if channels.contains(TIMER_IRQ) {
            if self.gpt.take_expiry() {
                TIMER_FROM_I2C.notify();
            }
            TIMER_IRQ.irq_ack().unwrap();
        }
        Ok(())
    }
}
