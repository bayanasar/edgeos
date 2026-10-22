// SPDX-License-Identifier: BSD-3-Clause
//! The i.MX I2C controller in a protection domain of its own. It holds the
//! controller's register page and nothing else: no pin multiplexing, no
//! clocks, no other device. Its one client asks for transfers by protected
//! procedure call; the bytes are copied in message registers.

#![no_std]
#![no_main]

use core::ptr::NonNull;
use core::time::Duration;

use pd_common::{CounterClock, ERR_REQUEST, I2C_FROM_BMP280, OP_I2C_TRANSFER};
use sel4_microkit::{
    Channel, Handler, Infallible, MessageInfo, debug_println, memory_region_symbol,
    protection_domain, with_msg_regs, with_msg_regs_mut,
};
use sensor_core::bus::{Clock, I2c};
use sensor_imx_i2c::{I2DR, IFDR_24MHZ_100KHZ, ImxI2c, Regs};
use sensor_wire::{MAX_READ, REPLY_WORDS, REQUEST_WORDS, pack, request_from_words, status_label};

/// The longest a client may keep this domain busy with one transfer. It
/// polls at the highest priority, so the cap protects every domain below it
/// from a client's distant deadline.
const TRANSFER_CAP: Duration = Duration::from_millis(100);

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
        "i2c-imx: controller page mapped, counter at {} Hz",
        clock.hz()
    );
    HandlerImpl {
        i2c: ImxI2c::new(regs, clock, IFDR_24MHZ_100KHZ),
        clock,
    }
}

struct HandlerImpl {
    i2c: ImxI2c<Mmio, CounterClock>,
    clock: CounterClock,
}

impl Handler for HandlerImpl {
    type Error = Infallible;

    fn protected(
        &mut self,
        channel: Channel,
        msg_info: MessageInfo,
    ) -> Result<MessageInfo, Self::Error> {
        let n = msg_info.count();
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
        if result.is_err() {
            return Ok(MessageInfo::new(status_label(result), 0));
        }
        let mut out = [0u64; REPLY_WORDS];
        let k = pack(read, &mut out);
        with_msg_regs_mut(|mrs| mrs[..k].copy_from_slice(&out[..k]));
        Ok(MessageInfo::new(0, k))
    }
}
