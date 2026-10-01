// SPDX-License-Identifier: BSD-3-Clause
//! A fake clock and a fake I2C bus for driver tests.
//!
//! The I2C bus routes each transfer to a [`FakeDevice`] by address.
//! [`RegisterDevice`] models the common register-pointer chip; a per-device
//! hook sees every register write, so a test can model what the chip does in
//! response (start a conversion, reset). Chips that behave differently
//! implement [`FakeDevice`] themselves.

#![forbid(unsafe_code)]

use std::cell::Cell;
use std::time::Duration;

use sensor_core::bus::{BusError, Clock, I2c, Instant};

/// A fake bus panics after this many transfers, so a driver that ignores its
/// deadline fails its test instead of filling memory.
pub const RUNAWAY: usize = 100_000;

/// Time moves only when a driver sleeps.
#[derive(Debug, Default)]
pub struct FakeClock {
    now: Cell<i64>,
}

impl FakeClock {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn advance(&self, d: Duration) {
        self.now.set(self.now.get() + d.as_nanos() as i64);
    }
}

impl Clock for FakeClock {
    fn now(&self) -> Instant {
        Instant::from_nanos(self.now.get())
    }

    fn sleep_until(&self, t: Instant) {
        if t.as_nanos() > self.now.get() {
            self.now.set(t.as_nanos());
        }
    }
}

/// A device on the fake bus. It sees each transfer addressed to it.
pub trait FakeDevice {
    fn addr(&self) -> u8;
    fn transfer(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), BusError>;
}

/// Called after each register write with the register, the value and the
/// device's register file.
pub type WriteHook = Box<dyn FnMut(u8, u8, &mut [u8; 256])>;

/// A chip with a register pointer: a write sets the pointer from its first
/// byte and stores the rest, a read returns bytes from the pointer, both
/// auto-incrementing.
pub struct RegisterDevice {
    pub addr: u8,
    pub regs: [u8; 256],
    ptr: u8,
    hook: Option<WriteHook>,
}

impl RegisterDevice {
    pub fn new(addr: u8) -> Self {
        RegisterDevice {
            addr,
            regs: [0; 256],
            ptr: 0,
            hook: None,
        }
    }

    pub fn with_hook(mut self, hook: WriteHook) -> Self {
        self.hook = Some(hook);
        self
    }
}

impl FakeDevice for RegisterDevice {
    fn addr(&self) -> u8 {
        self.addr
    }

    fn transfer(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), BusError> {
        if let Some((&reg, data)) = write.split_first() {
            self.ptr = reg;
            for &v in data {
                let r = self.ptr;
                self.regs[usize::from(r)] = v;
                if let Some(hook) = self.hook.as_mut() {
                    hook(r, v, &mut self.regs);
                }
                self.ptr = self.ptr.wrapping_add(1);
            }
        }
        for b in read.iter_mut() {
            *b = self.regs[usize::from(self.ptr)];
            self.ptr = self.ptr.wrapping_add(1);
        }
        Ok(())
    }
}

/// One completed transfer, for assertions.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Transfer {
    pub addr: u8,
    pub write: Vec<u8>,
    pub read_len: usize,
}

#[derive(Default)]
pub struct FakeI2c {
    pub devices: Vec<Box<dyn FakeDevice>>,
    pub log: Vec<Transfer>,
    /// Returned by the next transfer instead of performing it.
    pub fail_next: Option<BusError>,
}

impl FakeI2c {
    pub fn new(devices: Vec<Box<dyn FakeDevice>>) -> Self {
        FakeI2c {
            devices,
            ..Self::default()
        }
    }

    pub fn with_device(dev: impl FakeDevice + 'static) -> Self {
        Self::new(vec![Box::new(dev)])
    }

    /// Register writes seen so far, as (register, value) pairs. Meaningful
    /// for register-pointer devices only.
    pub fn register_writes(&self, addr: u8) -> Vec<(u8, u8)> {
        self.log
            .iter()
            .filter(|t| t.addr == addr && t.write.len() > 1)
            .flat_map(|t| {
                t.write[1..]
                    .iter()
                    .enumerate()
                    .map(move |(i, v)| (t.write[0] + i as u8, *v))
            })
            .collect()
    }
}

impl I2c for FakeI2c {
    fn transfer(
        &mut self,
        addr: u8,
        write: &[u8],
        read: &mut [u8],
        _deadline: Instant,
    ) -> Result<(), BusError> {
        if addr > 0x7f || (write.is_empty() && read.is_empty()) {
            return Err(BusError::Invalid);
        }
        if let Some(e) = self.fail_next.take() {
            return Err(e);
        }
        assert!(
            self.log.len() < RUNAWAY,
            "runaway: {RUNAWAY} transfers without stopping"
        );
        let dev = self
            .devices
            .iter_mut()
            .find(|d| d.addr() == addr)
            .ok_or(BusError::Nak)?;
        dev.transfer(write, read)?;
        self.log.push(Transfer {
            addr,
            write: write.to_vec(),
            read_len: read.len(),
        });
        Ok(())
    }
}

/// Answers one 1-Wire transaction: the bytes written after the reset, and a
/// buffer for the bytes to read back.
pub type OneWireResponder = Box<dyn FnMut(&[u8], &mut [u8]) -> Result<(), BusError>>;

/// A 1-Wire bus with at most one device, modelled by a responder. With no
/// responder the reset sees no presence pulse.
#[derive(Default)]
pub struct FakeOneWire {
    pub device: Option<OneWireResponder>,
    /// Bytes written by each transaction, and how many were read back.
    pub log: Vec<(Vec<u8>, usize)>,
}

impl FakeOneWire {
    pub fn with_device(responder: OneWireResponder) -> Self {
        FakeOneWire {
            device: Some(responder),
            log: Vec::new(),
        }
    }
}

impl sensor_core::bus::OneWire for FakeOneWire {
    fn transaction(
        &mut self,
        write: &[u8],
        read: &mut [u8],
        _deadline: Instant,
    ) -> Result<(), BusError> {
        assert!(
            self.log.len() < RUNAWAY,
            "runaway: {RUNAWAY} transactions without stopping"
        );
        let dev = self.device.as_mut().ok_or(BusError::NoPresence)?;
        dev(write, read)?;
        self.log.push((write.to_vec(), read.len()));
        Ok(())
    }
}
