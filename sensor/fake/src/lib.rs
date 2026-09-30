// SPDX-License-Identifier: BSD-3-Clause
//! A fake clock and a fake I2C bus for driver tests.
//!
//! The I2C bus holds register-file devices: a write sets the register pointer
//! from its first byte and stores the rest, a read returns bytes from the
//! pointer, both auto-incrementing, which is how most sensor chips behave.
//! A per-device hook sees every register write, so a test can model what the
//! chip does in response (start a conversion, reset).

#![forbid(unsafe_code)]

use std::cell::Cell;
use std::time::Duration;

use sensor_core::bus::{BusError, Clock, I2c, Instant};

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

/// Called after each register write with the register, the value and the
/// device's register file.
pub type WriteHook = Box<dyn FnMut(u8, u8, &mut [u8; 256])>;

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

/// One completed transfer, for assertions.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Transfer {
    pub addr: u8,
    pub write: Vec<u8>,
    pub read_len: usize,
}

#[derive(Default)]
pub struct FakeI2c {
    pub devices: Vec<RegisterDevice>,
    pub log: Vec<Transfer>,
    /// Returned by the next transfer instead of performing it.
    pub fail_next: Option<BusError>,
}

impl FakeI2c {
    pub fn new(devices: Vec<RegisterDevice>) -> Self {
        FakeI2c {
            devices,
            ..Self::default()
        }
    }

    pub fn device(&mut self, addr: u8) -> &mut RegisterDevice {
        self.devices
            .iter_mut()
            .find(|d| d.addr == addr)
            .expect("no fake device at address")
    }

    /// Register writes seen so far, as (register, value) pairs.
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
        if addr > 0x7f {
            return Err(BusError::Invalid);
        }
        if let Some(e) = self.fail_next.take() {
            return Err(e);
        }
        let dev = self
            .devices
            .iter_mut()
            .find(|d| d.addr == addr)
            .ok_or(BusError::Nak)?;
        if let Some((&reg, data)) = write.split_first() {
            dev.ptr = reg;
            for &v in data {
                let r = dev.ptr;
                dev.regs[usize::from(r)] = v;
                if let Some(hook) = dev.hook.as_mut() {
                    hook(r, v, &mut dev.regs);
                }
                dev.ptr = dev.ptr.wrapping_add(1);
            }
        }
        for b in read.iter_mut() {
            *b = dev.regs[usize::from(dev.ptr)];
            dev.ptr = dev.ptr.wrapping_add(1);
        }
        self.log.push(Transfer {
            addr,
            write: write.to_vec(),
            read_len: read.len(),
        });
        Ok(())
    }
}
