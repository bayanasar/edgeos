// SPDX-License-Identifier: BSD-3-Clause
//! GPIO through the kernel's character device, uAPI v2 (`linux/gpio.h`).
//!
//! Each configured line is its own line request, so a line can be
//! reconfigured without touching the others. Reconfiguring a line that is
//! already requested changes it in place (`GPIO_V2_LINE_SET_CONFIG_IOCTL`):
//! no release, so no window in which the line floats or another consumer can
//! take it, and one system call for an output-to-input turnaround. A line
//! switched to output drives low until it is set. Edge events carry the kernel's
//! timestamp, taken when it handles the interrupt, on `CLOCK_MONOTONIC`: the
//! same clock as [`MonotonicClock`]. A gap in a line's sequence number means
//! the kernel dropped events because its buffer was full; that is reported as
//! [`BusError::Overflow`].
//!
//! Each line has its own kernel queue, so a wait reads every ready queue
//! whole and merges the events by timestamp before handing out the oldest;
//! the rest are kept for the next call. Reading part of one queue could hand
//! a line's later edges to the caller before another line's earlier ones,
//! and a quadrature decoder would count those in the wrong direction. The
//! kernel stamps an edge in its interrupt handler and queues it from a
//! thread, so the order holds up to that thread's latency.

use std::collections::{BTreeMap, VecDeque};
use std::fs::{File, OpenOptions};
use std::io;
use std::mem;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};

use sensor_core::bus::{
    Bias, BusError, Clock, Direction, Gpio, GpioEvent, Instant, LineConfig, TsSource,
};

use crate::{MonotonicClock, errno_to_bus};

const LINES_MAX: usize = 64;
const NAME_SIZE: usize = 32;
const NUM_ATTRS_MAX: usize = 10;

const FLAG_INPUT: u64 = 1 << 2;
const FLAG_OUTPUT: u64 = 1 << 3;
const FLAG_EDGE_RISING: u64 = 1 << 4;
const FLAG_EDGE_FALLING: u64 = 1 << 5;
const FLAG_BIAS_PULL_UP: u64 = 1 << 8;
const FLAG_BIAS_PULL_DOWN: u64 = 1 << 9;
const FLAG_BIAS_DISABLED: u64 = 1 << 10;

const EVENT_RISING_EDGE: u32 = 1;

/// Events the kernel buffers per line before it starts dropping the oldest.
/// A DHT frame is up to 85 edges (release, response, 40 bits, end), all of
/// which arrive before the reader runs; the kernel's FIFO size is a power of
/// two, so 128 is the first size that holds a frame.
const EVENT_BUFFER: u32 = 128;
// The kernel rounds its queue up to a power of two; one read of this many
// events empties it only if no rounding happened.
const _: () = assert!(EVENT_BUFFER.is_power_of_two());
const CONSUMER: &[u8] = b"sensor";

// The kernel's structures. Every `__aligned_u64` member makes its structure
// 8-byte aligned on all targets, including i686, where a plain u64 is not.

#[repr(C, align(8))]
#[derive(Clone, Copy)]
struct LineAttribute {
    id: u32,
    padding: u32,
    value: u64,
}

#[repr(C, align(8))]
#[derive(Clone, Copy)]
struct ConfigAttribute {
    attr: LineAttribute,
    mask: u64,
}

#[repr(C, align(8))]
struct LineConfigRaw {
    flags: u64,
    num_attrs: u32,
    padding: [u32; 5],
    attrs: [ConfigAttribute; NUM_ATTRS_MAX],
}

#[repr(C, align(8))]
struct LineRequest {
    offsets: [u32; LINES_MAX],
    consumer: [u8; NAME_SIZE],
    config: LineConfigRaw,
    num_lines: u32,
    event_buffer_size: u32,
    padding: [u32; 5],
    fd: i32,
}

#[repr(C, align(8))]
struct LineValues {
    bits: u64,
    mask: u64,
}

#[repr(C, align(8))]
#[derive(Clone, Copy, Default)]
struct LineEvent {
    timestamp_ns: u64,
    id: u32,
    offset: u32,
    seqno: u32,
    line_seqno: u32,
    padding: [u32; 6],
}

// Sizes and offsets as printed from linux/gpio.h by a C compiler.
const _: () = assert!(mem::size_of::<LineAttribute>() == 16);
const _: () = assert!(mem::size_of::<ConfigAttribute>() == 24);
const _: () = assert!(mem::size_of::<LineConfigRaw>() == 272);
const _: () = assert!(mem::size_of::<LineRequest>() == 592);
const _: () = assert!(mem::offset_of!(LineRequest, config) == 288);
const _: () = assert!(mem::offset_of!(LineRequest, num_lines) == 560);
const _: () = assert!(mem::offset_of!(LineRequest, fd) == 588);
const _: () = assert!(mem::size_of::<LineValues>() == 16);
const _: () = assert!(mem::size_of::<LineEvent>() == 48);

/// `_IOWR(0xB4, nr, size)` in the generic encoding used by x86, Arm and
/// RISC-V.
const fn iowr(nr: u32, size: usize) -> u32 {
    (3 << 30) | ((size as u32) << 16) | (0xB4 << 8) | nr
}

const GET_LINE_IOCTL: u32 = iowr(0x07, mem::size_of::<LineRequest>());
const GET_VALUES_IOCTL: u32 = iowr(0x0E, mem::size_of::<LineValues>());
const SET_VALUES_IOCTL: u32 = iowr(0x0F, mem::size_of::<LineValues>());
const SET_CONFIG_IOCTL: u32 = iowr(0x0D, mem::size_of::<LineConfigRaw>());

const _: () = assert!(GET_LINE_IOCTL == 0xC250_B407);
const _: () = assert!(GET_VALUES_IOCTL == 0xC010_B40E);
const _: () = assert!(SET_VALUES_IOCTL == 0xC010_B40F);
const _: () = assert!(SET_CONFIG_IOCTL == 0xC110_B40D);

const ZERO_ATTR: ConfigAttribute = ConfigAttribute {
    attr: LineAttribute {
        id: 0,
        padding: 0,
        value: 0,
    },
    mask: 0,
};

/// A line configuration with no per-line attributes: every requested line
/// takes `flags`, and an output starts low.
fn line_config(flags: u64) -> LineConfigRaw {
    LineConfigRaw {
        flags,
        num_attrs: 0,
        padding: [0; 5],
        attrs: [ZERO_ATTR; NUM_ATTRS_MAX],
    }
}

/// The request flags for a line configuration. Edges are only meaningful on
/// an input.
fn line_flags(c: LineConfig) -> Result<u64, BusError> {
    let mut f = match c.direction {
        Direction::Input => FLAG_INPUT,
        Direction::Output if c.edges.rising || c.edges.falling => return Err(BusError::Invalid),
        Direction::Output => FLAG_OUTPUT,
    };
    if c.edges.rising {
        f |= FLAG_EDGE_RISING;
    }
    if c.edges.falling {
        f |= FLAG_EDGE_FALLING;
    }
    f |= match c.bias {
        Bias::None => FLAG_BIAS_DISABLED,
        Bias::PullUp => FLAG_BIAS_PULL_UP,
        Bias::PullDown => FLAG_BIAS_PULL_DOWN,
    };
    Ok(f)
}

/// The edge an event reports, and whether its line sequence number shows
/// events dropped since `last` (0 before the first event).
fn convert(e: &LineEvent, last: u32) -> (GpioEvent, bool) {
    let lost = last != 0 && e.line_seqno != last.wrapping_add(1);
    let ev = GpioEvent {
        timestamp: e.timestamp_ns as i64,
        line: e.offset,
        level: u8::from(e.id == EVENT_RISING_EDGE),
        ts_source: TsSource::Interrupt as u8,
        reserved: 0,
    };
    (ev, lost)
}

struct Line {
    req: OwnedFd,
    config: LineConfig,
    /// Sequence number of the last event read; 0 before the first.
    last_seqno: u32,
}

impl Line {
    fn reports_edges(&self) -> bool {
        self.config.direction == Direction::Input
            && (self.config.edges.rising || self.config.edges.falling)
    }
}

/// Adds events read from the kernel to `staged`, keeping it oldest first.
/// The sort is stable, so a line's own events keep their queue order.
fn stage(staged: &mut VecDeque<GpioEvent>, batch: &[GpioEvent]) {
    staged.extend(batch);
    staged.make_contiguous().sort_by_key(|e| e.timestamp);
}

/// Moves the oldest staged events into `out` and returns how many.
fn deliver(staged: &mut VecDeque<GpioEvent>, out: &mut [GpioEvent]) -> usize {
    let n = staged.len().min(out.len());
    for (o, e) in out.iter_mut().zip(staged.drain(..n)) {
        *o = e;
    }
    n
}

/// One GPIO controller, such as `/dev/gpiochip0`.
pub struct LinuxGpio {
    chip: File,
    lines: BTreeMap<u32, Line>,
    clock: MonotonicClock,
    /// Events read from the kernel and not yet handed out, oldest first.
    staged: VecDeque<GpioEvent>,
}

impl LinuxGpio {
    pub fn open(path: &str) -> io::Result<Self> {
        let chip = OpenOptions::new().read(true).write(true).open(path)?;
        Ok(LinuxGpio {
            chip,
            lines: BTreeMap::new(),
            clock: MonotonicClock,
            staged: VecDeque::new(),
        })
    }

    fn request(&self, offset: u32, flags: u64) -> Result<OwnedFd, BusError> {
        let mut consumer = [0u8; NAME_SIZE];
        consumer[..CONSUMER.len()].copy_from_slice(CONSUMER);
        let mut offsets = [0u32; LINES_MAX];
        offsets[0] = offset;
        let mut req = LineRequest {
            offsets,
            consumer,
            config: line_config(flags),
            num_lines: 1,
            event_buffer_size: EVENT_BUFFER,
            padding: [0; 5],
            fd: -1,
        };
        // SAFETY: `req` is a complete, initialised gpio_v2_line_request; the
        // kernel writes only its `fd` member.
        let rc = unsafe { libc::ioctl(self.chip.as_raw_fd(), GET_LINE_IOCTL as _, &mut req) };
        if rc < 0 {
            return Err(errno_to_bus(io::Error::last_os_error().raw_os_error()));
        }
        // SAFETY: on success the kernel returned a new descriptor we own.
        Ok(unsafe { OwnedFd::from_raw_fd(req.fd) })
    }

    fn values(&self, line: u32, ioctl: u32, bits: u64) -> Result<u64, BusError> {
        let l = self.lines.get(&line).ok_or(BusError::Invalid)?;
        let mut v = LineValues { bits, mask: 1 };
        // SAFETY: `v` is a gpio_v2_line_values the kernel reads or fills.
        let rc = unsafe { libc::ioctl(l.req.as_raw_fd(), ioctl as _, &mut v) };
        if rc < 0 {
            return Err(errno_to_bus(io::Error::last_os_error().raw_os_error()));
        }
        Ok(v.bits)
    }
}

impl Gpio for LinuxGpio {
    fn configure(&mut self, line: u32, config: LineConfig) -> Result<(), BusError> {
        let flags = line_flags(config)?;
        if let Some(l) = self.lines.get_mut(&line) {
            let mut c = line_config(flags);
            // SAFETY: `c` is a complete gpio_v2_line_config the kernel reads.
            let rc = unsafe { libc::ioctl(l.req.as_raw_fd(), SET_CONFIG_IOCTL as _, &mut c) };
            if rc < 0 {
                return Err(errno_to_bus(io::Error::last_os_error().raw_os_error()));
            }
            // The line keeps its request, its event FIFO and its sequence
            // numbers, so `last_seqno` stays valid.
            l.config = config;
            return Ok(());
        }
        let req = self.request(line, flags)?;
        self.lines.insert(
            line,
            Line {
                req,
                config,
                last_seqno: 0,
            },
        );
        Ok(())
    }

    fn set(&mut self, line: u32, high: bool) -> Result<(), BusError> {
        match self.lines.get(&line) {
            Some(l) if l.config.direction == Direction::Output => {}
            _ => return Err(BusError::Invalid),
        }
        self.values(line, SET_VALUES_IOCTL, u64::from(high))
            .map(|_| ())
    }

    fn get(&mut self, line: u32) -> Result<bool, BusError> {
        Ok(self.values(line, GET_VALUES_IOCTL, 0)? & 1 != 0)
    }

    fn wait_edges(&mut self, out: &mut [GpioEvent], deadline: Instant) -> Result<usize, BusError> {
        let watched: Vec<u32> = self
            .lines
            .iter()
            .filter(|(_, l)| l.reports_edges())
            .map(|(&n, _)| n)
            .collect();
        if out.is_empty() || watched.is_empty() {
            return Err(BusError::Invalid);
        }
        let mut fds: Vec<libc::pollfd> = watched
            .iter()
            .map(|n| libc::pollfd {
                fd: self.lines[n].req.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            })
            .collect();
        loop {
            // With events already staged, only collect what is ready now.
            let left = if self.staged.is_empty() {
                deadline.saturating_duration_since(self.clock.now())
            } else {
                std::time::Duration::ZERO
            };
            let ts = libc::timespec {
                tv_sec: left.as_secs() as _,
                tv_nsec: left.subsec_nanos() as _,
            };
            // SAFETY: `fds` is a live array of `fds.len()` pollfd structures.
            let rc =
                unsafe { libc::ppoll(fds.as_mut_ptr(), fds.len() as _, &ts, std::ptr::null()) };
            if rc < 0 {
                let err = io::Error::last_os_error();
                if err.kind() == io::ErrorKind::Interrupted {
                    continue;
                }
                return Err(errno_to_bus(err.raw_os_error()));
            }
            if rc == 0 && self.staged.is_empty() {
                return Err(BusError::Timeout);
            }
            break;
        }

        let mut lost = false;
        let mut buf = [LineEvent::default(); EVENT_BUFFER as usize];
        let mut batch = Vec::new();
        for (pfd, line) in fds.iter().zip(&watched) {
            if pfd.revents & (libc::POLLERR | libc::POLLHUP | libc::POLLNVAL) != 0 {
                return Err(BusError::Io);
            }
            if pfd.revents & libc::POLLIN == 0 {
                continue;
            }
            let l = self.lines.get_mut(line).expect("watched lines exist");
            // SAFETY: reads at most `buf.len()` whole events into `buf`. The
            // queue holds no more than that, so this one read empties it.
            let got = unsafe {
                libc::read(
                    l.req.as_raw_fd(),
                    buf.as_mut_ptr().cast(),
                    mem::size_of_val(&buf),
                )
            };
            if got < 0 {
                return Err(errno_to_bus(io::Error::last_os_error().raw_os_error()));
            }
            for e in &buf[..got as usize / mem::size_of::<LineEvent>()] {
                let (ev, gap) = convert(e, l.last_seqno);
                l.last_seqno = e.line_seqno;
                lost |= gap;
                batch.push(ev);
            }
        }
        if lost {
            // The caller resynchronises from the lines' levels now, so
            // nothing older may be handed out after this.
            self.staged.clear();
            return Err(BusError::Overflow);
        }
        stage(&mut self.staged, &batch);
        Ok(deliver(&mut self.staged, out))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sensor_core::bus::Edges;

    #[test]
    fn configurations_map_to_request_flags() {
        let input = LineConfig {
            direction: Direction::Input,
            bias: Bias::PullUp,
            edges: Edges::BOTH,
        };
        assert_eq!(
            line_flags(input),
            Ok(FLAG_INPUT | FLAG_EDGE_RISING | FLAG_EDGE_FALLING | FLAG_BIAS_PULL_UP)
        );
        let output = LineConfig {
            direction: Direction::Output,
            bias: Bias::None,
            edges: Edges::NONE,
        };
        assert_eq!(line_flags(output), Ok(FLAG_OUTPUT | FLAG_BIAS_DISABLED));
        let bad = LineConfig {
            edges: Edges::RISING,
            ..output
        };
        assert_eq!(line_flags(bad), Err(BusError::Invalid));
    }

    #[test]
    fn events_convert_and_gaps_are_detected() {
        let e = |id, line_seqno| LineEvent {
            timestamp_ns: 1_000,
            id,
            offset: 24,
            seqno: line_seqno,
            line_seqno,
            padding: [0; 6],
        };
        let (ev, lost) = convert(&e(1, 1), 0);
        assert_eq!(
            (ev.timestamp, ev.line, ev.level, lost),
            (1_000, 24, 1, false)
        );
        assert_eq!(ev.ts_source, TsSource::Interrupt as u8);
        assert_eq!(convert(&e(2, 2), 1).0.level, 0);
        assert!(!convert(&e(2, 2), 1).1);
        assert!(convert(&e(2, 4), 2).1, "event 3 was dropped");
        assert!(!convert(&e(1, 0), u32::MAX).1, "the counter wraps");
    }

    #[test]
    fn edges_are_handed_out_oldest_first_across_lines() {
        let ev = |line, t| GpioEvent {
            timestamp: t,
            line,
            level: 1,
            ts_source: TsSource::Interrupt as u8,
            reserved: 0,
        };
        // Two lines' queues, read one after the other: line 5 in full, then
        // line 6, whose edges interleave with line 5's.
        let mut staged = VecDeque::new();
        let first: Vec<_> = (0..20).map(|i| ev(5, 10 * i)).collect();
        let second: Vec<_> = (0..20).map(|i| ev(6, 10 * i + 5)).collect();
        stage(&mut staged, &first);
        stage(&mut staged, &second);
        let mut out = [GpioEvent::default(); 8];
        let mut seen = Vec::new();
        while !staged.is_empty() {
            let n = deliver(&mut staged, &mut out);
            seen.extend(out[..n].iter().map(|e| (e.line, e.timestamp)));
        }
        let want: Vec<_> = (0..40)
            .map(|i| (5 + (i % 2) as u32, 5 * i as i64))
            .collect();
        assert_eq!(seen, want);
        // Equal timestamps keep the order in which they were read.
        stage(&mut staged, &[ev(6, 7), ev(5, 7)]);
        assert_eq!(deliver(&mut staged, &mut out), 2);
        assert_eq!((out[0].line, out[1].line), (6, 5));
    }

    #[test]
    fn waiting_needs_a_line_that_reports_edges() {
        let mut g = LinuxGpio {
            chip: File::open("/dev/null").unwrap(),
            lines: BTreeMap::new(),
            clock: MonotonicClock,
            staged: VecDeque::new(),
        };
        let mut ev = [GpioEvent::default(); 1];
        assert_eq!(
            g.wait_edges(&mut ev, MonotonicClock.now()),
            Err(BusError::Invalid)
        );
        assert_eq!(g.set(17, true), Err(BusError::Invalid));
        assert_eq!(g.get(17), Err(BusError::Invalid));
        // /dev/null is not a GPIO chip: the request fails without a line.
        let out = LineConfig {
            direction: Direction::Output,
            bias: Bias::None,
            edges: Edges::NONE,
        };
        assert!(g.configure(17, out).is_err());
        assert!(g.lines.is_empty());
    }
}
