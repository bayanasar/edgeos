// SPDX-License-Identifier: BSD-3-Clause
//! The i.MX I2C controller in a protection domain of its own. It holds the
//! controller's register page and interrupt and nothing else: no pin
//! multiplexing, no clocks, no other device. Its one client asks for
//! transfers by protected procedure call; the bytes are copied in message
//! registers. While a byte is on the bus the domain blocks until the
//! controller interrupts or the timer's domain signals the deadline.

#![no_std]
#![no_main]

use core::ptr::NonNull;
use core::time::Duration;

use pd_common::{
    CounterClock, ERR_REQUEST, I2C_FROM_BMP280, I2C_IRQ, I2C_TO_TIMER, OP_I2C_REPORT,
    OP_I2C_TRANSFER, Timeout, TimerClient,
};
use sel4_microkit::{
    Channel, ChannelSet, Handler, Infallible, MessageInfo, debug_println, memory_region_symbol,
    protection_domain, with_msg_regs, with_msg_regs_mut,
};
use sensor_core::bus::{BusError, Clock, I2c, Instant};
use sensor_imx_i2c::{I2DR, IFDR_24MHZ_100KHZ, ImxI2c, Regs, Wait};
use sensor_wire::{MAX_READ, REPLY_WORDS, REQUEST_WORDS, pack, request_from_words, status_label};

/// The longest one transfer may hold the bus, whatever deadline the client
/// sends: what Linux allows each byte (`i2c_imx_trx_complete`), and about 20
/// times the longest transfer the wire format allows, 50 bytes or 4.5 ms at
/// 100 kHz.
const TRANSFER_CAP: Duration = Duration::from_millis(100);

/// This domain's endpoint, where Microkit's event loop receives: the slot of
/// Microkit's `INPUT_CAP`, which neither the SDK nor `sel4-microkit`
/// exports. `build.sh` checks that both still put it in slot 1.
const INPUT: sel4::cap::Endpoint = sel4::Cap::from_bits(1);

/// `seL4_Wait` on [`INPUT`]: returns the badge of the notifications that
/// woke it. The `sel4` crate offers `wait` on notification capabilities
/// only, but the kernel takes an endpoint as well (`handleRecv` without a
/// reply object), which is what is wanted here.
fn wait_input() -> sel4::Badge {
    INPUT.cast::<sel4::cap_type::Notification>().wait().1
}

/// Blocks on [`INPUT`] for the controller's interrupt or the timer's signal.
///
/// A protected procedure cannot return to Microkit's event loop to wait, so
/// it waits on the endpoint itself, with `seL4_Wait`: no reply object, so the
/// reply owed to the caller is kept, and the notifications bound to the
/// domain still arrive. This is sound only because the domain has one
/// client, blocked in the call being served: a call arriving now would find
/// no reply object and its caller would be left inactive (seL4's
/// `sendIPC`). A second client needs a bus multiplexer in front of this
/// domain, not a second channel into it.
struct IrqWait {
    timer: TimerClient,
    deadline: Deadline,
    /// The interrupt was delivered and is masked until acknowledged, which
    /// waits until the driver has cleared the flag that raised it.
    ack_pending: bool,
    counts: Counts,
}

/// What the timer's domain said about the deadline last sent to it.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Deadline {
    /// None since the last transfer, or its signal has come.
    Unset,
    /// A signal will come at this deadline.
    Armed(Instant),
    /// The timer's clock had passed this deadline, a tick or so before
    /// ours: the driver polls the rest of the way, without asking again.
    Passed(Instant),
    /// The timer does not count. The timer's domain decides that once, at
    /// its start, so the driver polls from then on without asking again.
    Stopped,
}

/// How each wait ended, so a run shows which path it took. A byte that
/// completed before the driver looked at the status is not waited for, and
/// not counted.
#[derive(Default)]
struct Counts {
    /// Woken by the controller's interrupt.
    irq: u32,
    /// Woken by the timer's signal: the deadline, or an early signal that
    /// the clock overrules.
    timer: u32,
    /// Returned without blocking, for the driver to poll, because the timer
    /// could not help.
    polled: u32,
}

impl IrqWait {
    fn ack(&mut self) {
        if self.ack_pending {
            I2C_IRQ.irq_ack().unwrap();
            self.ack_pending = false;
        }
    }

    /// Lets the driver look at the status again, without blocking.
    fn poll(&mut self) -> Result<(), BusError> {
        self.counts.polled += 1;
        Ok(())
    }
}

impl Wait for IrqWait {
    const INTERRUPT: bool = true;

    fn wait(&mut self, clock: &impl Clock, deadline: Instant) -> Result<(), BusError> {
        self.ack();
        loop {
            if clock.now() >= deadline {
                return Err(BusError::Timeout);
            }
            match self.deadline {
                Deadline::Armed(d) if d == deadline => {}
                Deadline::Passed(d) if d == deadline => return self.poll(),
                Deadline::Stopped => return self.poll(),
                _ => match self.timer.set(deadline) {
                    Timeout::Armed => self.deadline = Deadline::Armed(deadline),
                    Timeout::Passed => {
                        self.deadline = Deadline::Passed(deadline);
                        return self.poll();
                    }
                    Timeout::Stopped => {
                        self.deadline = Deadline::Stopped;
                        return self.poll();
                    }
                },
            }
            let badge = wait_input();
            if badge & (1 << I2C_TO_TIMER.index()) != 0 {
                self.counts.timer += 1;
                // Possibly a deadline already cancelled; the clock decides.
                self.deadline = Deadline::Unset;
            }
            if badge & (1 << I2C_IRQ.index()) != 0 {
                self.counts.irq += 1;
                self.ack_pending = true;
                return Ok(());
            }
        }
    }

    fn finish(&mut self) {
        self.ack();
        match self.deadline {
            Deadline::Armed(_) => self.timer.cancel(),
            Deadline::Stopped => return,
            _ => {}
        }
        self.deadline = Deadline::Unset;
    }
}

/// The register page, mapped uncached.
struct Mmio(NonNull<u8>);

impl Regs for Mmio {
    fn read(&mut self, offset: usize) -> u8 {
        assert!(offset <= I2DR);
        // SAFETY: the page is mapped for this domain alone and the offset is
        // one of the controller's registers.
        unsafe { self.0.as_ptr().add(offset).read_volatile() }
    }

    fn write(&mut self, offset: usize, value: u8) {
        assert!(offset <= I2DR);
        // SAFETY: as above.
        unsafe { self.0.as_ptr().add(offset).write_volatile(value) }
    }
}

#[protection_domain]
fn init() -> HandlerImpl {
    let regs = Mmio(memory_region_symbol!(i2c_regs: *mut u8));
    let clock = CounterClock::new();
    debug_println!(
        "i2c-imx: controller page mapped, counter at {} Hz, interrupt-driven",
        clock.hz()
    );
    let wait = IrqWait {
        timer: TimerClient::new(I2C_TO_TIMER),
        deadline: Deadline::Unset,
        ack_pending: false,
        counts: Counts::default(),
    };
    HandlerImpl {
        i2c: ImxI2c::with_wait(regs, clock, IFDR_24MHZ_100KHZ, wait),
        clock,
        transfers: 0,
    }
}

struct HandlerImpl {
    i2c: ImxI2c<Mmio, CounterClock, IrqWait>,
    clock: CounterClock,
    transfers: u32,
}

impl HandlerImpl {
    /// Prints the transfers so far and how their waits ended.
    fn report(&self) {
        let c = &self.i2c.waiter().counts;
        debug_println!(
            "I2C_WAITS: transfers={} irq={} timer={} polled={}",
            self.transfers,
            c.irq,
            c.timer,
            c.polled
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
        let n = msg_info.count();
        if channel == I2C_FROM_BMP280 && msg_info.label() == OP_I2C_REPORT && n == 0 {
            self.report();
            return Ok(MessageInfo::new(0, 0));
        }
        if channel != I2C_FROM_BMP280 || msg_info.label() != OP_I2C_TRANSFER || n > REQUEST_WORDS {
            return Ok(MessageInfo::new(ERR_REQUEST, 0));
        }
        let mut words = [0u64; REQUEST_WORDS];
        with_msg_regs(|mrs| words[..n].copy_from_slice(&mrs[..n]));
        let req = match request_from_words(&words[..n]) {
            Ok(req) => req,
            Err(e) => return Ok(MessageInfo::new(status_label(Err(e)), 0)),
        };
        let cap = self.clock.now().checked_add(TRANSFER_CAP);
        let deadline = cap.map_or(req.deadline, |cap| cap.min(req.deadline));
        let mut buf = [0u8; MAX_READ];
        let read = &mut buf[..req.read_len];
        let result = self.i2c.transfer(req.addr, req.write(), read, deadline);
        self.transfers += 1;
        if result.is_err() {
            return Ok(MessageInfo::new(status_label(result), 0));
        }
        let mut out = [0u64; REPLY_WORDS];
        let k = pack(read, &mut out);
        with_msg_regs_mut(|mrs| mrs[..k].copy_from_slice(&out[..k]));
        Ok(MessageInfo::new(0, k))
    }

    /// Notifications that arrive between transfers: the interrupt of a byte
    /// that completed before the driver needed to wait for it, or the signal
    /// of a deadline that was cancelled after it fired. The block is
    /// disabled between transfers, so its interrupt line is low and the
    /// interrupt can be acknowledged; the deadline needs nothing.
    fn notified(&mut self, channels: ChannelSet) -> Result<(), Self::Error> {
        if channels.contains(I2C_IRQ) {
            I2C_IRQ.irq_ack().unwrap();
        }
        Ok(())
    }
}
