// SPDX-License-Identifier: BSD-3-Clause
//! The general-purpose timer (GPT) of the i.MX 8M family (`fsl,imx8mp-gpt`,
//! compatible with `fsl,imx6dl-gpt`) as a one-shot timeout source, on its
//! first output compare.
//!
//! The configuration is Linux's `drivers/clocksource/timer-imx-gpt.c` for
//! the i.MX 6DL and later: a free-running counter on the 24 MHz oscillator
//! divided by 8, which Linux gives as 3 MHz. The driver does not rely on that
//! rate: [`ImxGpt::calibrate`] measures it against a clock, so a different
//! clock source or a model that ignores the oscillator prescaler gives
//! correct timeouts too. Registers are 32 bits wide; the status flags clear
//! when written as 1.
//!
//! The timer must already be clocked; its clock gates belong to the rest of
//! the chip.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::time::Duration;
use sensor_core::bus::Clock;

pub const CR: usize = 0x00;
pub const PR: usize = 0x04;
pub const SR: usize = 0x08;
pub const IR: usize = 0x0c;
pub const OCR1: usize = 0x10;
pub const CNT: usize = 0x24;

pub const CR_EN: u32 = 1 << 0;
pub const CR_WAITEN: u32 = 1 << 3;
pub const CR_CLKSRC_SHIFT: u32 = 6;
pub const CR_CLKSRC_MASK: u32 = 7 << CR_CLKSRC_SHIFT;
/// The 24 MHz oscillator, through the oscillator prescaler.
pub const CR_CLKSRC_OSC: u32 = 5 << CR_CLKSRC_SHIFT;
pub const CR_FRR: u32 = 1 << 9;
pub const CR_EN_24M: u32 = 1 << 10;

pub const PR_PRESCALER_MASK: u32 = 0xfff;
pub const PR_PRESCALER24M_SHIFT: u32 = 12;

pub const SR_OF1: u32 = 1 << 0;
pub const IR_OF1IE: u32 = 1 << 0;

/// Word access to the timer's registers, at the offsets above.
pub trait Regs {
    fn read(&mut self, offset: usize) -> u32;
    fn write(&mut self, offset: usize, value: u32);
}

impl<T: Regs + ?Sized> Regs for &mut T {
    fn read(&mut self, offset: usize) -> u32 {
        (**self).read(offset)
    }

    fn write(&mut self, offset: usize, value: u32) {
        (**self).write(offset, value)
    }
}

/// The outcome of arming the compare.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Arm {
    /// The timer interrupts when the counter reaches the compare value.
    Armed,
    /// The counter passed the compare value before it was set; nothing will
    /// interrupt until the counter wraps, so the compare is disarmed.
    Passed,
}

pub struct ImxGpt<R> {
    regs: R,
    hz: u64,
    armed: bool,
}

impl<R: Regs> ImxGpt<R> {
    /// Resets the timer and starts it counting as Linux does, with the
    /// compare disarmed. The rate is unknown until [`calibrate`] runs.
    ///
    /// [`calibrate`]: ImxGpt::calibrate
    pub fn start(mut regs: R) -> Self {
        regs.write(CR, 0);
        regs.write(IR, 0);
        regs.write(PR, 7 << PR_PRESCALER24M_SHIFT);
        regs.write(CR, CR_FRR | CR_WAITEN | CR_CLKSRC_OSC | CR_EN_24M | CR_EN);
        regs.write(SR, SR_OF1);
        ImxGpt {
            regs,
            hz: 0,
            armed: false,
        }
    }

    pub fn release(self) -> R {
        self.regs
    }

    pub fn count(&mut self) -> u32 {
        self.regs.read(CNT)
    }

    /// Measures the counting rate over at least `over` of `clock`, and keeps
    /// it for [`ticks_for`]. `None` if the counter did not move.
    ///
    /// [`ticks_for`]: ImxGpt::ticks_for
    pub fn calibrate(&mut self, clock: &impl Clock, over: Duration) -> Option<u64> {
        let t0 = clock.now();
        let c0 = self.count();
        let end = t0.checked_add(over)?;
        while clock.now() < end {}
        let c1 = self.count();
        let ns = clock.now().saturating_duration_since(t0).as_nanos();
        let ticks = u128::from(c1.wrapping_sub(c0));
        if ticks == 0 || ns == 0 {
            return None;
        }
        self.hz = u64::try_from(ticks * 1_000_000_000 / ns).ok()?;
        Some(self.hz)
    }

    pub fn hz(&self) -> u64 {
        self.hz
    }

    /// Ticks that cover `d`, rounded up: at least 1, and less than half the
    /// counter's range, so a compare value is always ahead of the counter.
    pub fn ticks_for(&self, d: Duration) -> u32 {
        let ticks = (d.as_nanos() * u128::from(self.hz)).div_ceil(1_000_000_000);
        ticks.clamp(1, i32::MAX as u128) as u32
    }

    /// Interrupts once the counter is `ticks` past its current value.
    pub fn arm(&mut self, ticks: u32) -> Arm {
        let ticks = ticks.clamp(1, i32::MAX as u32);
        self.regs.write(IR, 0);
        let target = self.count().wrapping_add(ticks);
        self.regs.write(OCR1, target);
        // Cleared after the new value is in place, so a match of the old one
        // cannot pass for this one.
        self.regs.write(SR, SR_OF1);
        self.regs.write(IR, IR_OF1IE);
        self.armed = true;
        // Linux's v2_set_next_event: a compare value the counter has already
        // reached matches only after the counter wraps. A match cleared above
        // is among them, so the check is against the counter, not the flag.
        let ahead = target.wrapping_sub(self.count()) as i32;
        if ahead <= 0 && self.regs.read(SR) & SR_OF1 == 0 {
            self.disarm();
            return Arm::Passed;
        }
        Arm::Armed
    }

    pub fn disarm(&mut self) {
        self.regs.write(IR, 0);
        self.regs.write(SR, SR_OF1);
        self.armed = false;
    }

    /// Whether the compare matched since it was armed. A match clears and
    /// disarms it. The flag also sets while disarmed, each time the counter
    /// wraps round to the old compare value, so it counts only when armed.
    pub fn take_expiry(&mut self) -> bool {
        if !self.armed || self.regs.read(SR) & SR_OF1 == 0 {
            return false;
        }
        self.disarm();
        true
    }
}
