// SPDX-License-Identifier: BSD-3-Clause
//! The general-purpose timer (GPT) of the i.MX 8M family (`fsl,imx8mp-gpt`,
//! compatible with `fsl,imx6dl-gpt`) as one-shot timeout sources, one on
//! each of its three output compares. The compares share the counter and
//! the interrupt line, and each arms, matches and disarms apart from the
//! others.
//!
//! The configuration is Linux's `drivers/clocksource/timer-imx-gpt.c` for
//! the i.MX 6DL and later: a free-running counter on the 24 MHz oscillator
//! divided by 8, which Linux gives as 3 MHz. The driver does not rely on that
//! rate: [`ImxGpt::calibrate`] measures it against a clock, so a different
//! clock source or a model that ignores the oscillator prescaler gives
//! correct timeouts too. Registers are 32 bits wide; the status flags clear
//! when written as 1. In free-run mode no compare resets the counter, the
//! first included.
//!
//! The timer must already be clocked; its clock gates belong to the rest of
//! the chip.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use core::time::Duration;
use sensor_core::bus::{Clock, Instant};

pub const CR: usize = 0x00;
pub const PR: usize = 0x04;
pub const SR: usize = 0x08;
pub const IR: usize = 0x0c;
pub const OCR1: usize = 0x10;
pub const OCR2: usize = 0x14;
pub const OCR3: usize = 0x18;
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

/// Output compare flags in SR, and the matching interrupt enables in IR,
/// which use the same bits.
pub const SR_OF1: u32 = 1 << 0;
pub const SR_OF2: u32 = 1 << 1;
pub const SR_OF3: u32 = 1 << 2;
pub const IR_OF1IE: u32 = 1 << 0;
pub const IR_OF2IE: u32 = 1 << 1;
pub const IR_OF3IE: u32 = 1 << 2;

/// One of the timer's three output compares.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Compare {
    One,
    Two,
    Three,
}

impl Compare {
    pub const ALL: [Compare; 3] = [Compare::One, Compare::Two, Compare::Three];

    /// Its compare register.
    pub const fn ocr(self) -> usize {
        match self {
            Compare::One => OCR1,
            Compare::Two => OCR2,
            Compare::Three => OCR3,
        }
    }

    /// Its bit in SR and in IR.
    pub const fn flag(self) -> u32 {
        match self {
            Compare::One => SR_OF1,
            Compare::Two => SR_OF2,
            Compare::Three => SR_OF3,
        }
    }
}

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

/// The outcome of arming a compare.
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
    /// The flags of the armed compares: what IR holds, so arming or
    /// disarming one compare writes the others' enables back unchanged.
    armed: u32,
}

impl<R: Regs> ImxGpt<R> {
    /// Resets the timer and starts it counting as Linux does, with every
    /// compare disarmed. The rate is unknown until [`calibrate`] runs.
    ///
    /// [`calibrate`]: ImxGpt::calibrate
    pub fn start(mut regs: R) -> Self {
        regs.write(CR, 0);
        regs.write(IR, 0);
        regs.write(PR, 7 << PR_PRESCALER24M_SHIFT);
        regs.write(CR, CR_FRR | CR_WAITEN | CR_CLKSRC_OSC | CR_EN_24M | CR_EN);
        regs.write(SR, SR_OF1 | SR_OF2 | SR_OF3);
        ImxGpt {
            regs,
            hz: 0,
            armed: 0,
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
    /// Each of the two counts is timed by the clock readings either side of
    /// it, so the time a register read takes is not counted into the
    /// interval. Counted in, it would measure the rate low by that share of
    /// `over`: 0.1 % for a read that takes 1 us, over 1 ms.
    ///
    /// That takes each count to be sampled midway between its clock readings.
    /// A read that follows register writes is not: on the i.MX 8M Plus it
    /// waits about 1.4 us for them and returns a count sampled near the end
    /// of the wait, up to 1 us after the midpoint. [`start`] writes registers
    /// just before this runs, so one count is read and dropped first;
    /// counted, it measured the rate 0.06 % to 0.17 % low over 1 ms.
    ///
    /// [`ticks_for`]: ImxGpt::ticks_for
    /// [`start`]: ImxGpt::start
    pub fn calibrate(&mut self, clock: &impl Clock, over: Duration) -> Option<u64> {
        // Waits out any register writes before the timed reads.
        self.count();
        let before0 = clock.now();
        let c0 = self.count();
        let after0 = clock.now();
        let end = before0.checked_add(over)?;
        while clock.now() < end {}
        let before1 = clock.now();
        let c1 = self.count();
        let after1 = clock.now();
        // Twice the time between the midpoints of the two readings.
        let sum = |a: Instant, b: Instant| i128::from(a.as_nanos()) + i128::from(b.as_nanos());
        let ns2 = u128::try_from(sum(before1, after1) - sum(before0, after0)).ok()?;
        let ticks = u128::from(c1.wrapping_sub(c0));
        if ticks == 0 || ns2 == 0 {
            return None;
        }
        self.hz = u64::try_from(ticks * 2_000_000_000 / ns2).ok()?;
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

    /// Interrupts once the counter is `ticks` past its current value, on
    /// compare `c`. The other compares are left as they are.
    pub fn arm(&mut self, c: Compare, ticks: u32) -> Arm {
        let ticks = ticks.clamp(1, i32::MAX as u32);
        let flag = c.flag();
        self.armed &= !flag;
        self.regs.write(IR, self.armed);
        let target = self.count().wrapping_add(ticks);
        self.regs.write(c.ocr(), target);
        // Cleared after the new value is in place, so a match of the old one
        // cannot pass for this one.
        self.regs.write(SR, flag);
        self.armed |= flag;
        self.regs.write(IR, self.armed);
        // Linux's v2_set_next_event: a compare value the counter has already
        // reached matches only after the counter wraps. A match cleared above
        // is among them, so the check is against the counter, not the flag.
        let ahead = target.wrapping_sub(self.count()) as i32;
        if ahead <= 0 && self.regs.read(SR) & flag == 0 {
            self.disarm(c);
            return Arm::Passed;
        }
        Arm::Armed
    }

    pub fn disarm(&mut self, c: Compare) {
        self.armed &= !c.flag();
        self.regs.write(IR, self.armed);
        self.regs.write(SR, c.flag());
    }

    /// Whether compare `c` matched since it was armed. A match clears and
    /// disarms it. The flag also sets while disarmed, each time the counter
    /// wraps round to the old compare value, so it counts only when armed.
    pub fn take_expiry(&mut self, c: Compare) -> bool {
        if self.armed & c.flag() == 0 || self.regs.read(SR) & c.flag() == 0 {
            return false;
        }
        self.disarm(c);
        true
    }
}
