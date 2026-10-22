// SPDX-License-Identifier: BSD-3-Clause
//! The I2C controller of the i.MX 8M family (`fsl,imx21-i2c`) as a polled
//! [`I2c`] master.
//!
//! The register sequence is Linux's `drivers/i2c/busses/i2c-imx.c` in its
//! atomic mode: polled, single master, no DMA. Registers are 8 bits wide and
//! 4 bytes apart, and the status flags clear when written as 0. Register
//! access goes through [`Regs`], so the same code drives the hardware from an
//! isolated component and a register model in tests.
//!
//! The controller must already be clocked and its pads muxed to I2C; neither
//! is this driver's to touch, since both registers are shared with the rest
//! of the chip.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::time::Duration;
use sensor_core::bus::{BusError, Clock, I2c, Instant};

pub const IADR: usize = 0x00;
pub const IFDR: usize = 0x04;
pub const I2CR: usize = 0x08;
pub const I2SR: usize = 0x0c;
pub const I2DR: usize = 0x10;

pub const I2CR_IEN: u8 = 0x80;
pub const I2CR_IIEN: u8 = 0x40;
pub const I2CR_MSTA: u8 = 0x20;
pub const I2CR_MTX: u8 = 0x10;
pub const I2CR_TXAK: u8 = 0x08;
pub const I2CR_RSTA: u8 = 0x04;

pub const I2SR_ICF: u8 = 0x80;
pub const I2SR_IBB: u8 = 0x20;
pub const I2SR_IAL: u8 = 0x10;
pub const I2SR_IIF: u8 = 0x02;
pub const I2SR_RXAK: u8 = 0x01;

/// IFDR for 100 kHz from a 24 MHz root clock: divider 240, `{ 240, 0x0F }`
/// in Linux's `imx_i2c_clk_div`.
pub const IFDR_24MHZ_100KHZ: u8 = 0x0f;

/// Linux waits 50 us after enabling the block before taking the bus.
const ENABLE_SETTLE: Duration = Duration::from_micros(50);
/// Budget for the stop that ends every transfer, counted from when it is
/// issued, so a transfer that timed out still releases the bus. A stop takes
/// about 10 us at 100 kHz.
const STOP_LIMIT: Duration = Duration::from_millis(1);

/// Byte access to the controller's registers, at the offsets above.
pub trait Regs {
    fn read(&mut self, offset: usize) -> u8;
    fn write(&mut self, offset: usize, value: u8);
}

impl<T: Regs + ?Sized> Regs for &mut T {
    fn read(&mut self, offset: usize) -> u8 {
        (**self).read(offset)
    }

    fn write(&mut self, offset: usize, value: u8) {
        (**self).write(offset, value)
    }
}

pub struct ImxI2c<R, C> {
    regs: R,
    clock: C,
    ifdr: u8,
}

impl<R: Regs, C: Clock> ImxI2c<R, C> {
    /// `ifdr` selects the bus clock from the controller's root clock, for
    /// example [`IFDR_24MHZ_100KHZ`].
    pub fn new(regs: R, clock: C, ifdr: u8) -> Self {
        ImxI2c { regs, clock, ifdr }
    }

    pub fn release(self) -> (R, C) {
        (self.regs, self.clock)
    }

    fn control(&mut self) -> u8 {
        self.regs.read(I2CR)
    }

    fn set_control(&mut self, value: u8) {
        self.regs.write(I2CR, value)
    }

    /// Clears the given status flags, as Linux's `i2c_imx_clear_irq` does.
    /// On this controller IIF and IAL clear when written as 0 and the other
    /// bits are read-only, so writing 1 everywhere else changes nothing.
    /// (Vybrid's controller clears by writing 1; it is not supported.)
    fn clear_flags(&mut self, flags: u8) {
        self.regs.write(I2SR, !flags)
    }

    /// Waits until the bus-busy flag equals `busy`. Lost arbitration means
    /// another master or a stuck line: an I/O error, not a retry.
    fn wait_bus(&mut self, busy: bool, deadline: Instant) -> Result<(), BusError> {
        loop {
            let sr = self.regs.read(I2SR);
            if sr & I2SR_IAL != 0 {
                return Err(BusError::Io);
            }
            if (sr & I2SR_IBB != 0) == busy {
                return Ok(());
            }
            if self.clock.now() >= deadline {
                return Err(BusError::Timeout);
            }
        }
    }

    /// Waits for the byte in flight, clears its flag and returns the status.
    fn wait_byte(&mut self, deadline: Instant) -> Result<u8, BusError> {
        loop {
            let sr = self.regs.read(I2SR);
            if sr & I2SR_IIF != 0 {
                self.clear_flags(I2SR_IIF | I2SR_IAL);
                if sr & I2SR_IAL != 0 {
                    return Err(BusError::Io);
                }
                return Ok(sr);
            }
            if self.clock.now() >= deadline {
                return Err(BusError::Timeout);
            }
        }
    }

    /// Sends one byte in transmit mode; the receiver must acknowledge it.
    fn send(&mut self, byte: u8, deadline: Instant) -> Result<(), BusError> {
        self.regs.write(I2DR, byte);
        if self.wait_byte(deadline)? & I2SR_RXAK != 0 {
            return Err(BusError::Nak);
        }
        Ok(())
    }

    fn start(&mut self, deadline: Instant) -> Result<(), BusError> {
        self.regs.write(IFDR, self.ifdr);
        self.clear_flags(I2SR_IIF | I2SR_IAL);
        self.set_control(I2CR_IEN);
        if let Some(t) = self.clock.now().checked_add(ENABLE_SETTLE) {
            self.clock.sleep_until(t);
        }
        let control = self.control() | I2CR_MSTA;
        self.set_control(control);
        self.wait_bus(true, deadline)?;
        self.set_control(control | I2CR_MTX | I2CR_TXAK);
        Ok(())
    }

    /// Issues a stop if the controller still owns the bus, then disables it.
    fn stop(&mut self) -> Result<(), BusError> {
        let control = self.control();
        let mut result = Ok(());
        if control & I2CR_MSTA != 0 {
            self.set_control(control & !(I2CR_MSTA | I2CR_MTX));
            let limit = self.clock.now().checked_add(STOP_LIMIT);
            result = self.wait_bus(false, limit.unwrap_or(Instant::from_nanos(i64::MAX)));
        }
        self.set_control(0);
        result
    }

    fn read_bytes(&mut self, addr: u8, buf: &mut [u8], deadline: Instant) -> Result<(), BusError> {
        self.send((addr << 1) | 1, deadline)?;
        let n = buf.len();
        let mut control = self.control() & !I2CR_MTX;
        if n > 1 {
            // Acknowledge every byte but the last; start() left TXAK set.
            control &= !I2CR_TXAK;
        }
        self.set_control(control);
        // In receive mode each read of I2DR clocks in the next byte, so this
        // first read only starts the transfer.
        let _ = self.regs.read(I2DR);
        for (i, slot) in buf.iter_mut().enumerate() {
            self.wait_byte(deadline)?;
            if i + 1 == n {
                // Stop before reading the last byte, so that reading it does
                // not clock in another.
                let control = self.control();
                self.set_control(control & !(I2CR_MSTA | I2CR_MTX));
                self.wait_bus(false, deadline)?;
            } else if i + 2 == n {
                // The byte now arriving is the last one: refuse it.
                let control = self.control();
                self.set_control(control | I2CR_TXAK);
            }
            *slot = self.regs.read(I2DR);
        }
        Ok(())
    }

    fn exchange(
        &mut self,
        addr: u8,
        write: &[u8],
        read: &mut [u8],
        deadline: Instant,
    ) -> Result<(), BusError> {
        if !write.is_empty() {
            self.send(addr << 1, deadline)?;
            for &b in write {
                self.send(b, deadline)?;
            }
            if !read.is_empty() {
                let control = self.control();
                self.set_control(control | I2CR_RSTA);
                self.wait_bus(true, deadline)?;
            }
        }
        if !read.is_empty() {
            self.read_bytes(addr, read, deadline)?;
        }
        Ok(())
    }
}

impl<R: Regs, C: Clock> I2c for ImxI2c<R, C> {
    fn transfer(
        &mut self,
        addr: u8,
        write: &[u8],
        read: &mut [u8],
        deadline: Instant,
    ) -> Result<(), BusError> {
        // 7-bit addresses 0x00 to 0x07 and 0x78 to 0x7f are reserved: general
        // call, start byte, CBUS, high-speed mode and 10-bit addressing
        // (I2C-bus specification UM10204, section 3.1.12).
        if !(0x08..=0x77).contains(&addr) || (write.is_empty() && read.is_empty()) {
            return Err(BusError::Invalid);
        }
        let result = self
            .start(deadline)
            .and_then(|()| self.exchange(addr, write, read, deadline));
        let stopped = self.stop();
        result.and(stopped)
    }
}
