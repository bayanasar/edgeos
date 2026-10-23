// SPDX-License-Identifier: BSD-3-Clause
//! What the sensor protection domains share: channel numbers and labels from
//! `rsb3720.system`, the generic timer as a clock, an I2C client that calls
//! the controller's domain, and a timeout client that calls the timer's.

#![no_std]

use core::arch::asm;

use sel4_microkit::{Channel, MessageInfo, with_msg_regs, with_msg_regs_mut};
use sensor_bmp280::Error;
use sensor_core::bus::{BusError, Clock, I2c, Instant};
use sensor_wire::{REQUEST_WORDS, request_to_words, status_from_label, unpack, words_for};

/// In the BMP280 domain: its channel to the I2C controller's domain.
pub const BMP280_TO_I2C: Channel = Channel::new(0);
/// In the I2C controller's domain: the BMP280 domain's channel.
pub const I2C_FROM_BMP280: Channel = Channel::new(0);
/// In the consumer: its channel to the BMP280 domain.
pub const CONSUMER_TO_BMP280: Channel = Channel::new(0);
/// In the BMP280 domain: the consumer's channel.
pub const BMP280_FROM_CONSUMER: Channel = Channel::new(1);
/// In the I2C controller's domain: the controller's interrupt.
pub const I2C_IRQ: Channel = Channel::new(1);
/// In the I2C controller's domain: its channel to the timer's domain.
pub const I2C_TO_TIMER: Channel = Channel::new(2);
/// In the timer's domain: the timer's interrupt.
pub const TIMER_IRQ: Channel = Channel::new(0);
/// In the timer's domain: the I2C controller's domain, its one client.
pub const TIMER_FROM_I2C: Channel = Channel::new(1);

/// Label of an I2C transfer request; the reply's label is its status.
pub const OP_I2C_TRANSFER: u64 = 1;
/// Label of a request for one sample; the reply's label is
/// [`sample_status`].
pub const OP_READ_SAMPLE: u64 = 1;
/// Label of a request that the controller's domain print how its waits have
/// ended (`I2C_WAITS:`); the reply is empty. Printing at 115200 baud takes
/// about 87 us a character, so it is asked for once, after the timed
/// requests, not done in each transfer.
pub const OP_I2C_REPORT: u64 = 2;
/// Label of a request that the BMP280 domain pass [`OP_I2C_REPORT`] on to the
/// controller's domain; the reply is empty.
pub const OP_REPORT: u64 = 2;
/// Label of a request for a signal at a deadline, in nanoseconds of
/// [`CounterClock`] in the first word. It replaces any earlier deadline.
pub const OP_TIMEOUT_SET: u64 = 2;
/// Label of a request to drop the deadline.
pub const OP_TIMEOUT_CANCEL: u64 = 3;
/// Reply label: no signal will come, because the deadline has passed.
pub const TIMEOUT_PASSED: u64 = 1;
/// Reply label: the timer is not counting, so no signal will ever come.
pub const TIMER_STOPPED: u64 = 2;
/// Reply label: the request was not understood.
pub const ERR_REQUEST: u64 = 0xffff;

/// A BMP280 outcome as a reply label: 0 for a sample, otherwise
/// 0x100 + bus error code, 0x200 | chip ID, or 0x300 and up.
pub fn sample_status(e: Option<Error>) -> u64 {
    match e {
        None => 0,
        Some(Error::Bus(b)) => 0x100 + b as u64,
        Some(Error::WrongChip(id)) => 0x200 | u64::from(id),
        Some(Error::BadCalibration) => 0x300,
        Some(Error::NotInitialized) => 0x301,
        Some(Error::NotReady) => 0x302,
        Some(Error::NoData) => 0x303,
        Some(Error::Clock) => 0x304,
    }
}

/// The generic timer's virtual counter, readable at EL0 because the kernel
/// is built with `KernelArmExportVCNTUser`. Every domain reading it shares
/// one timebase, so deadlines can cross domains.
#[derive(Clone, Copy)]
pub struct CounterClock {
    hz: u64,
}

impl CounterClock {
    pub fn new() -> Self {
        let hz: u64;
        // SAFETY: reading CNTFRQ_EL0 has no side effects; EL0 may read it
        // once the kernel enables EL0 access to the virtual counter.
        unsafe { asm!("mrs {}, cntfrq_el0", out(reg) hz) };
        CounterClock { hz }
    }

    pub fn hz(&self) -> u64 {
        self.hz
    }

    fn ticks() -> u64 {
        let t: u64;
        // SAFETY: as above; the ISB keeps the read from moving earlier.
        unsafe { asm!("isb", "mrs {}, cntvct_el0", out(reg) t) };
        t
    }
}

impl Default for CounterClock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for CounterClock {
    /// Whole seconds and the remainder are converted apart, in 64 bits: polling
    /// loops call this, and a 128-bit division is a library call. The
    /// remainder is below the counter's rate, so its product with 10^9 fits
    /// for any rate under 18 GHz.
    fn now(&self) -> Instant {
        let (t, hz) = (Self::ticks(), self.hz.max(1));
        let ns = t / hz * 1_000_000_000 + t % hz * 1_000_000_000 / hz;
        Instant::from_nanos(ns as i64)
    }

    /// Spins: these domains have no timer driver.
    fn sleep_until(&self, t: Instant) {
        while self.now() < t {
            core::hint::spin_loop();
        }
    }
}

/// [`I2c`] over protected procedure calls to the controller's domain. The
/// bytes travel in message registers, copied both ways.
pub struct I2cClient {
    channel: Channel,
}

impl I2cClient {
    pub const fn new(channel: Channel) -> Self {
        I2cClient { channel }
    }
}

impl I2c for I2cClient {
    fn transfer(
        &mut self,
        addr: u8,
        write: &[u8],
        read: &mut [u8],
        deadline: Instant,
    ) -> Result<(), BusError> {
        let mut words = [0u64; REQUEST_WORDS];
        let n = request_to_words(addr, write, read.len(), deadline, &mut words)?;
        with_msg_regs_mut(|mrs| mrs[..n].copy_from_slice(&words[..n]));
        let reply = self.channel.pp_call(MessageInfo::new(OP_I2C_TRANSFER, n));
        status_from_label(reply.label())?;
        let need = words_for(read.len());
        if reply.count() != need {
            return Err(BusError::Io);
        }
        with_msg_regs(|mrs| unpack(&mrs[..need], read));
        Ok(())
    }
}

/// What the timer's domain answered to a deadline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Timeout {
    /// A signal will come on the channel at the deadline.
    Armed,
    /// The deadline has passed; no signal will come.
    Passed,
    /// The timer does not count; no signal will ever come.
    Stopped,
}

/// Deadlines by protected procedure call to the timer's domain, which
/// signals back on the same channel.
pub struct TimerClient {
    channel: Channel,
}

impl TimerClient {
    pub const fn new(channel: Channel) -> Self {
        TimerClient { channel }
    }

    pub fn channel(&self) -> Channel {
        self.channel
    }

    pub fn set(&self, deadline: Instant) -> Timeout {
        with_msg_regs_mut(|mrs| mrs[0] = deadline.as_nanos() as u64);
        match self.channel.pp_call(MessageInfo::new(OP_TIMEOUT_SET, 1)).label() {
            0 => Timeout::Armed,
            TIMEOUT_PASSED => Timeout::Passed,
            _ => Timeout::Stopped,
        }
    }

    pub fn cancel(&self) {
        self.channel.pp_call(MessageInfo::new(OP_TIMEOUT_CANCEL, 0));
    }
}
