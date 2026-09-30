// SPDX-License-Identifier: BSD-3-Clause
//! 1-Wire through the kernel's w1 core, using its netlink connector protocol
//! (Linux `Documentation/w1/w1-netlink.rst`, `drivers/w1/w1_netlink.h`).
//!
//! One transaction is one `W1_MASTER_CMD` message carrying reset, write and
//! read commands. The kernel runs a message's commands in order while holding
//! the bus, so nothing else on the bus can interleave, and it answers each
//! command with a status reply and each read with a data reply.

use std::io;
use std::mem;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::time::Duration;

use sensor_core::bus::{BusError, Clock, Instant, OneWire};

use crate::MonotonicClock;

const NETLINK_CONNECTOR: libc::c_int = 11;
const NLMSG_DONE: u16 = 3;
const CN_W1_IDX: u32 = 3;
const CN_W1_VAL: u32 = 1;
const W1_MASTER_CMD: u8 = 4;
const W1_CMD_READ: u8 = 0;
const W1_CMD_WRITE: u8 = 1;
const W1_CMD_RESET: u8 = 5;

const NLMSG_HDR: usize = 16;
const CN_HDR: usize = 20;
const MSG_HDR: usize = 12;
const CMD_HDR: usize = 4;

/// One w1 bus master, `w1_bus_masterN` in sysfs.
pub struct LinuxW1 {
    sock: OwnedFd,
    master: u32,
    seq: u32,
    clock: MonotonicClock,
}

impl LinuxW1 {
    pub fn open(master: u32) -> io::Result<Self> {
        // SAFETY: plain socket creation; the result is checked before use.
        let fd = unsafe {
            libc::socket(
                libc::AF_NETLINK,
                libc::SOCK_DGRAM | libc::SOCK_CLOEXEC,
                NETLINK_CONNECTOR,
            )
        };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        // SAFETY: `fd` is a freshly created descriptor that nothing else owns.
        let sock = unsafe { OwnedFd::from_raw_fd(fd) };
        let addr = netlink_addr();
        // SAFETY: `addr` is a valid sockaddr_nl and the length matches it.
        let rc = unsafe {
            libc::bind(
                sock.as_raw_fd(),
                (&addr as *const libc::sockaddr_nl).cast(),
                mem::size_of::<libc::sockaddr_nl>() as libc::socklen_t,
            )
        };
        if rc < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(LinuxW1 {
            sock,
            master,
            seq: std::process::id().wrapping_mul(0x10000),
            clock: MonotonicClock,
        })
    }

    fn send(&self, msg: &[u8]) -> io::Result<()> {
        let kernel = netlink_addr();
        // SAFETY: `msg` and `kernel` are valid for the duration of the call.
        let rc = unsafe {
            libc::sendto(
                self.sock.as_raw_fd(),
                msg.as_ptr().cast(),
                msg.len(),
                0,
                (&kernel as *const libc::sockaddr_nl).cast(),
                mem::size_of::<libc::sockaddr_nl>() as libc::socklen_t,
            )
        };
        if rc < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(())
        }
    }

    /// Waits for a datagram until the deadline; `None` on timeout.
    fn recv(&self, buf: &mut [u8], deadline: Instant) -> Result<Option<usize>, BusError> {
        let left = deadline.saturating_duration_since(self.clock.now());
        if left.is_zero() {
            return Ok(None);
        }
        let ms = left.as_millis().clamp(1, i32::MAX as u128) as libc::c_int;
        let mut pfd = libc::pollfd {
            fd: self.sock.as_raw_fd(),
            events: libc::POLLIN,
            revents: 0,
        };
        // SAFETY: one valid pollfd.
        let rc = unsafe { libc::poll(&mut pfd, 1, ms) };
        if rc < 0 {
            return Err(BusError::Io);
        }
        if rc == 0 {
            return Ok(None);
        }
        let mut from = netlink_addr();
        let mut from_len = mem::size_of::<libc::sockaddr_nl>() as libc::socklen_t;
        // SAFETY: `buf` is writable for its full length; `from` and
        // `from_len` describe a writable sockaddr_nl.
        let n = unsafe {
            libc::recvfrom(
                self.sock.as_raw_fd(),
                buf.as_mut_ptr().cast(),
                buf.len(),
                0,
                (&mut from as *mut libc::sockaddr_nl).cast(),
                &mut from_len,
            )
        };
        if n < 0 {
            return Err(BusError::Io);
        }
        // Only the kernel (port 0) speaks for the w1 core; drop anything else.
        Ok(Some(if from.nl_pid == 0 { n as usize } else { 0 }))
    }
}

fn netlink_addr() -> libc::sockaddr_nl {
    // SAFETY: sockaddr_nl is plain data for which all-zero is valid.
    let mut a: libc::sockaddr_nl = unsafe { mem::zeroed() };
    a.nl_family = libc::AF_NETLINK as libc::sa_family_t;
    a
}

/// Builds the request: netlink header, connector header, one master message
/// with its commands.
fn request(seq: u32, master: u32, write: &[u8], read_len: usize) -> Vec<u8> {
    let mut cmds = Vec::new();
    let mut cmd = |op: u8, data: &[u8]| {
        cmds.push(op);
        cmds.push(0);
        cmds.extend_from_slice(&(data.len() as u16).to_ne_bytes());
        cmds.extend_from_slice(data);
    };
    cmd(W1_CMD_RESET, &[]);
    if !write.is_empty() {
        cmd(W1_CMD_WRITE, write);
    }
    if read_len > 0 {
        cmd(W1_CMD_READ, &vec![0u8; read_len]);
    }
    let mut msg = vec![W1_MASTER_CMD, 0];
    msg.extend_from_slice(&(cmds.len() as u16).to_ne_bytes());
    msg.extend_from_slice(&master.to_ne_bytes());
    msg.extend_from_slice(&0u32.to_ne_bytes());
    msg.extend_from_slice(&cmds);

    let mut cn = Vec::with_capacity(CN_HDR + msg.len());
    cn.extend_from_slice(&CN_W1_IDX.to_ne_bytes());
    cn.extend_from_slice(&CN_W1_VAL.to_ne_bytes());
    cn.extend_from_slice(&seq.to_ne_bytes());
    cn.extend_from_slice(&0u32.to_ne_bytes()); // ack
    cn.extend_from_slice(&(msg.len() as u16).to_ne_bytes());
    cn.extend_from_slice(&0u16.to_ne_bytes()); // flags: no bundling
    cn.extend_from_slice(&msg);

    let len = NLMSG_HDR + cn.len();
    let mut nl = Vec::with_capacity(len.next_multiple_of(4));
    nl.extend_from_slice(&(len as u32).to_ne_bytes());
    nl.extend_from_slice(&NLMSG_DONE.to_ne_bytes());
    nl.extend_from_slice(&0u16.to_ne_bytes());
    nl.extend_from_slice(&seq.to_ne_bytes());
    nl.extend_from_slice(&0u32.to_ne_bytes());
    nl.extend_from_slice(&cn);
    nl.resize(len.next_multiple_of(4), 0);
    nl
}

/// What the replies so far have told us.
#[derive(Default, Debug, PartialEq, Eq)]
struct Replies {
    /// (command, status) for each command the kernel has finished.
    statuses: Vec<(u8, u8)>,
    data: Option<Vec<u8>>,
    /// Set when the kernel rejected the whole message (for example ENODEV
    /// for a master that does not exist); it then sends no command replies.
    message_error: Option<u8>,
}

const ENODEV: u8 = 19;

/// One status reply per command sent: reset, then write and read if present.
fn expected_statuses(write: &[u8], read: &[u8]) -> usize {
    1 + usize::from(!write.is_empty()) + usize::from(!read.is_empty())
}

/// Decides the transaction once the replies are complete: `None` while
/// command statuses are still outstanding.
fn outcome(r: &Replies, expected: usize, read: &mut [u8]) -> Option<Result<(), BusError>> {
    if let Some(e) = r.message_error {
        return Some(Err(if e == ENODEV {
            BusError::Invalid
        } else {
            BusError::Io
        }));
    }
    if r.statuses.len() < expected {
        return None;
    }
    for &(op, status) in &r.statuses {
        if status != 0 {
            // A reset with no presence pulse reports 1 from w1_reset_bus,
            // which the kernel sends as (u8)-1.
            return Some(Err(if op == W1_CMD_RESET {
                BusError::NoPresence
            } else {
                BusError::Io
            }));
        }
    }
    if !read.is_empty() {
        match &r.data {
            Some(d) if d.len() == read.len() => read.copy_from_slice(d),
            _ => return Some(Err(BusError::Io)),
        }
    }
    Some(Ok(()))
}

/// Parses one received datagram, keeping replies to `seq`.
fn parse(buf: &[u8], seq: u32, out: &mut Replies) {
    let u16_at = |b: &[u8], i: usize| u16::from_ne_bytes([b[i], b[i + 1]]) as usize;
    let u32_at = |b: &[u8], i: usize| u32::from_ne_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
    let mut nl = 0;
    while nl + NLMSG_HDR + CN_HDR <= buf.len() {
        let nl_len = u32_at(buf, nl) as usize;
        if nl_len < NLMSG_HDR + CN_HDR || nl_len > buf.len() - nl {
            return;
        }
        let cn = &buf[nl + NLMSG_HDR..nl + nl_len];
        let data_len = u16_at(cn, 16);
        let ours = u32_at(cn, 0) == CN_W1_IDX && u32_at(cn, 4) == CN_W1_VAL && u32_at(cn, 8) == seq;
        if ours && CN_HDR + data_len <= cn.len() {
            let data = &cn[CN_HDR..CN_HDR + data_len];
            let mut m = 0;
            while m + MSG_HDR <= data.len() {
                let (kind, status, len) = (data[m], data[m + 1], u16_at(data, m + 2));
                let body = &data[m + MSG_HDR..(m + MSG_HDR + len).min(data.len())];
                if kind == W1_MASTER_CMD && len == 0 && status != 0 {
                    out.message_error = Some(status);
                }
                if kind == W1_MASTER_CMD {
                    let mut c = 0;
                    while c + CMD_HDR <= body.len() {
                        let (op, clen) = (body[c], u16_at(body, c + 2));
                        let cdata = &body[c + CMD_HDR..(c + CMD_HDR + clen).min(body.len())];
                        if op == W1_CMD_READ && clen > 0 {
                            out.data = Some(cdata.to_vec());
                        } else if clen == 0 {
                            out.statuses.push((op, status));
                        }
                        c += CMD_HDR + clen;
                    }
                }
                m += MSG_HDR + len;
            }
        }
        nl += nl_len.next_multiple_of(4);
    }
}

impl OneWire for LinuxW1 {
    fn transaction(
        &mut self,
        write: &[u8],
        read: &mut [u8],
        deadline: Instant,
    ) -> Result<(), BusError> {
        if write.len() > 4096 || read.len() > 4096 {
            return Err(BusError::Invalid);
        }
        self.seq = self.seq.wrapping_add(1);
        let seq = self.seq;
        let expected = expected_statuses(write, read);
        self.send(&request(seq, self.master, write, read.len()))
            .map_err(|_| BusError::Io)?;

        let mut replies = Replies::default();
        let mut buf = vec![0u8; 16 * 1024];
        loop {
            if let Some(result) = outcome(&replies, expected, read) {
                return result;
            }
            match self.recv(&mut buf, deadline)? {
                Some(n) => parse(&buf[..n], seq, &mut replies),
                None => return Err(BusError::Timeout),
            }
        }
    }
}

/// Upper bound for one transaction, for callers that want a default.
pub const TRANSACTION_TIMEOUT: Duration = Duration::from_millis(500);

#[cfg(test)]
mod tests {
    use super::*;

    /// A reply datagram as the kernel builds it: one connector message with
    /// one master message and one command.
    fn reply(seq: u32, status: u8, op: u8, data: &[u8]) -> Vec<u8> {
        let mut cmd = vec![op, 0];
        cmd.extend_from_slice(&(data.len() as u16).to_ne_bytes());
        cmd.extend_from_slice(data);
        let mut msg = vec![W1_MASTER_CMD, status];
        msg.extend_from_slice(&(cmd.len() as u16).to_ne_bytes());
        msg.extend_from_slice(&[1, 0, 0, 0, 0, 0, 0, 0]);
        msg.extend_from_slice(&cmd);
        let mut cn = Vec::new();
        for v in [CN_W1_IDX, CN_W1_VAL, seq, 0] {
            cn.extend_from_slice(&v.to_ne_bytes());
        }
        cn.extend_from_slice(&(msg.len() as u16).to_ne_bytes());
        cn.extend_from_slice(&0u16.to_ne_bytes());
        cn.extend_from_slice(&msg);
        let len = NLMSG_HDR + cn.len();
        let mut nl = (len as u32).to_ne_bytes().to_vec();
        nl.extend_from_slice(&[0; 12]);
        nl.extend_from_slice(&cn);
        nl.resize(len.next_multiple_of(4), 0);
        nl
    }

    #[test]
    fn request_layout_matches_the_kernel_headers() {
        let r = request(7, 1, &[0xCC, 0xBE], 9);
        let len = u32::from_ne_bytes(r[0..4].try_into().unwrap()) as usize;
        // 16 nlmsghdr + 20 cn_msg + 12 w1_netlink_msg + (4) + (4+2) + (4+9)
        assert_eq!(len, 16 + 20 + 12 + 4 + 6 + 13);
        assert_eq!(r.len() % 4, 0);
        let cn = &r[16..];
        assert_eq!(u32::from_ne_bytes(cn[8..12].try_into().unwrap()), 7);
        assert_eq!(&cn[0..8], &[3, 0, 0, 0, 1, 0, 0, 0]); // CN_W1_IDX, CN_W1_VAL
        assert_eq!(&cn[18..20], &[0, 0], "no W1_CN_BUNDLE");
        assert_eq!(
            u16::from_ne_bytes(cn[16..18].try_into().unwrap()),
            12 + 4 + 6 + 13
        );
        let msg = &cn[20..];
        assert_eq!(
            (msg[0], u32::from_ne_bytes(msg[4..8].try_into().unwrap())),
            (W1_MASTER_CMD, 1)
        );
        let cmds = &msg[12..];
        assert_eq!(
            (cmds[0], cmds[4], cmds[10]),
            (W1_CMD_RESET, W1_CMD_WRITE, W1_CMD_READ)
        );
        assert_eq!(&cmds[8..10], &[0xCC, 0xBE]);
    }

    fn feed(r: &mut Replies, d: Vec<u8>) {
        parse(&d, 9, r);
    }

    #[test]
    fn the_transaction_completes_only_after_the_last_status() {
        // Kernel order for reset, write, read: status, status, data, status.
        let mut r = Replies::default();
        let mut read = [0u8; 3];
        feed(&mut r, reply(9, 0, W1_CMD_RESET, &[]));
        assert_eq!(outcome(&r, 3, &mut read), None);
        feed(&mut r, reply(9, 0, W1_CMD_WRITE, &[]));
        assert_eq!(outcome(&r, 3, &mut read), None);
        feed(&mut r, reply(9, 0, W1_CMD_READ, &[7, 8, 9]));
        assert_eq!(outcome(&r, 3, &mut read), None);
        feed(&mut r, reply(9, 0, W1_CMD_READ, &[]));
        assert_eq!(outcome(&r, 3, &mut read), Some(Ok(())));
        assert_eq!(read, [7, 8, 9]);
    }

    #[test]
    fn one_status_is_expected_per_command_sent() {
        assert_eq!(expected_statuses(&[0xCC, 0x44], &[]), 2);
        assert_eq!(expected_statuses(&[0xCC, 0xBE], &[0; 9]), 3);
        assert_eq!(expected_statuses(&[], &[]), 1);
        // The request carries exactly that many commands.
        let r = request(1, 1, &[0xCC, 0xBE], 9);
        assert_eq!(r[16 + 20 + 12], W1_CMD_RESET);
    }

    #[test]
    fn no_presence_and_command_errors_are_distinguished() {
        let mut r = Replies::default();
        feed(&mut r, reply(9, 255, W1_CMD_RESET, &[])); // (u8)-1 from w1_reset_bus
        feed(&mut r, reply(9, 0, W1_CMD_WRITE, &[]));
        assert_eq!(outcome(&r, 2, &mut []), Some(Err(BusError::NoPresence)));

        let mut r = Replies::default();
        feed(&mut r, reply(9, 0, W1_CMD_RESET, &[]));
        feed(&mut r, reply(9, 22, W1_CMD_WRITE, &[])); // EINVAL
        assert_eq!(outcome(&r, 2, &mut []), Some(Err(BusError::Io)));
    }

    #[test]
    fn missing_or_short_read_data_is_an_error() {
        let mut r = Replies::default();
        feed(&mut r, reply(9, 0, W1_CMD_RESET, &[]));
        feed(&mut r, reply(9, 0, W1_CMD_READ, &[]));
        assert_eq!(outcome(&r, 2, &mut [0; 2]), Some(Err(BusError::Io)));
        feed(&mut r, reply(9, 0, W1_CMD_READ, &[1]));
        assert_eq!(outcome(&r, 2, &mut [0; 2]), Some(Err(BusError::Io)));
    }

    #[test]
    fn a_rejected_message_fails_at_once() {
        // w1_netlink_send_error: the request's message, length 0, status set.
        let mut msg_only = reply(9, ENODEV, W1_CMD_RESET, &[]);
        let cmd_start = NLMSG_HDR + CN_HDR + MSG_HDR;
        msg_only.truncate(cmd_start);
        msg_only[NLMSG_HDR + CN_HDR + 2..NLMSG_HDR + CN_HDR + 4]
            .copy_from_slice(&0u16.to_ne_bytes());
        msg_only[NLMSG_HDR + 16..NLMSG_HDR + 18].copy_from_slice(&(MSG_HDR as u16).to_ne_bytes());
        msg_only[0..4].copy_from_slice(&(cmd_start as u32).to_ne_bytes());
        let mut r = Replies::default();
        feed(&mut r, msg_only);
        assert_eq!(outcome(&r, 3, &mut [0; 9]), Some(Err(BusError::Invalid)));
    }

    #[test]
    fn replies_are_sorted_into_statuses_and_data() {
        let mut r = Replies::default();
        let mut dgram = reply(9, 0, W1_CMD_RESET, &[]);
        dgram.extend(reply(9, 0, W1_CMD_READ, &[1, 2, 3]));
        dgram.extend(reply(9, 0, W1_CMD_READ, &[]));
        dgram.extend(reply(8, 0, W1_CMD_RESET, &[])); // another request's reply
        parse(&dgram, 9, &mut r);
        assert_eq!(r.statuses, [(W1_CMD_RESET, 0), (W1_CMD_READ, 0)]);
        assert_eq!(r.data, Some(vec![1, 2, 3]));
    }

    #[test]
    fn a_truncated_datagram_is_ignored_safely() {
        let mut r = Replies::default();
        let d = reply(3, 0, W1_CMD_RESET, &[]);
        for cut in 0..d.len() {
            parse(&d[..cut], 3, &mut r);
        }
        parse(&[0xff; 40], 3, &mut r);
        assert!(r.statuses.is_empty());
    }

    #[test]
    fn arbitrary_bytes_never_panic() {
        let mut x: u32 = 0x1234_5678;
        let mut r = Replies::default();
        for len in 0..400 {
            let buf: Vec<u8> = (0..len)
                .map(|_| {
                    x ^= x << 13;
                    x ^= x >> 17;
                    x ^= x << 5;
                    x as u8
                })
                .collect();
            parse(&buf, 3, &mut r);
            // A plausible header followed by garbage.
            let mut framed = reply(3, 0, W1_CMD_READ, &buf[..buf.len().min(64)]);
            let cut = framed.len() / 2 + len % 7;
            framed.truncate(cut.min(framed.len()));
            parse(&framed, 3, &mut r);
        }
    }
}
