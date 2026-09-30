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
        // SAFETY: `buf` is writable for its full length.
        let n = unsafe { libc::recv(self.sock.as_raw_fd(), buf.as_mut_ptr().cast(), buf.len(), 0) };
        if n < 0 {
            Err(BusError::Io)
        } else {
            Ok(Some(n as usize))
        }
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
    statuses: Vec<(u8, u8)>,
    data: Option<Vec<u8>>,
}

/// Parses one received datagram, keeping replies to `seq`.
fn parse(buf: &[u8], seq: u32, out: &mut Replies) {
    let u16_at = |b: &[u8], i: usize| u16::from_ne_bytes([b[i], b[i + 1]]) as usize;
    let u32_at = |b: &[u8], i: usize| u32::from_ne_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
    let mut nl = 0;
    while nl + NLMSG_HDR + CN_HDR <= buf.len() {
        let nl_len = u32_at(buf, nl) as usize;
        if nl_len < NLMSG_HDR + CN_HDR || nl + nl_len > buf.len() {
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
        let expected = 1 + usize::from(!write.is_empty()) + usize::from(!read.is_empty());
        self.send(&request(seq, self.master, write, read.len()))
            .map_err(|_| BusError::Io)?;

        let mut replies = Replies::default();
        let mut buf = vec![0u8; 16 * 1024];
        while replies.statuses.len() < expected {
            match self.recv(&mut buf, deadline)? {
                Some(n) => parse(&buf[..n], seq, &mut replies),
                None => return Err(BusError::Timeout),
            }
        }
        for &(op, status) in &replies.statuses {
            if status != 0 {
                return Err(if op == W1_CMD_RESET {
                    BusError::NoPresence
                } else {
                    BusError::Io
                });
            }
        }
        if !read.is_empty() {
            let data = replies.data.ok_or(BusError::Io)?;
            if data.len() != read.len() {
                return Err(BusError::Io);
            }
            read.copy_from_slice(&data);
        }
        Ok(())
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
