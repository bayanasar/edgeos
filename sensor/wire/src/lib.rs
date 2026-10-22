// SPDX-License-Identifier: BSD-3-Clause
//! Sensor records and I2C requests as 64-bit message words, for transports
//! that pass messages in registers, such as seL4 IPC.
//!
//! A [`Sample`] is copied whole into [`SAMPLE_WORDS`] words: its fields,
//! little-endian, at the byte offsets of its `repr(C)` layout. An I2C request
//! writes at most [`MAX_WRITE`] bytes and reads at most [`MAX_READ`]; its
//! status travels as the message label. Decoding checks every length and
//! does not trust the sender.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use sensor_core::bus::{BusError, Instant};
use sensor_core::sample::{Channel, MAX_CHANNELS, SAMPLE_SIZE, Sample};

pub const SAMPLE_WORDS: usize = SAMPLE_SIZE / 8;
pub const MAX_WRITE: usize = 16;
pub const MAX_READ: usize = 32;
/// The largest request: a header word, the deadline, the bytes to write.
pub const REQUEST_WORDS: usize = 2 + MAX_WRITE / 8;
/// The largest reply: the bytes read.
pub const REPLY_WORDS: usize = MAX_READ / 8;

/// Words needed for `n` bytes.
pub const fn words_for(n: usize) -> usize {
    n.div_ceil(8)
}

/// Packs `bytes` into the first `words_for(bytes.len())` words, little-endian.
/// Panics if `out` is too short.
pub fn pack(bytes: &[u8], out: &mut [u64]) -> usize {
    let n = words_for(bytes.len());
    for (w, chunk) in out[..n].iter_mut().zip(bytes.chunks(8)) {
        let mut b = [0u8; 8];
        b[..chunk.len()].copy_from_slice(chunk);
        *w = u64::from_le_bytes(b);
    }
    n
}

/// Fills `out` from the start of `words`. Panics if `words` is too short.
pub fn unpack(words: &[u64], out: &mut [u8]) {
    for (chunk, w) in out.chunks_mut(8).zip(words) {
        chunk.copy_from_slice(&w.to_le_bytes()[..chunk.len()]);
    }
}

pub fn sample_to_words(s: &Sample) -> [u64; SAMPLE_WORDS] {
    let mut b = [0u8; SAMPLE_SIZE];
    b[0..8].copy_from_slice(&s.timestamp.to_le_bytes());
    b[8..12].copy_from_slice(&s.sensor_id.to_le_bytes());
    b[12..16].copy_from_slice(&s.seq.to_le_bytes());
    b[16..20].copy_from_slice(&s.epoch.to_le_bytes());
    b[20..22].copy_from_slice(&s.kind.to_le_bytes());
    b[22..24].copy_from_slice(&s.flags.to_le_bytes());
    b[24] = s.ts_source;
    b[25] = s.nchannels;
    b[26..28].copy_from_slice(&s.reserved0.to_le_bytes());
    b[28..32].copy_from_slice(&s.reserved1.to_le_bytes());
    for (c, o) in s.ch.iter().zip((32..).step_by(12)) {
        b[o..o + 4].copy_from_slice(&c.raw.to_le_bytes());
        b[o + 4..o + 8].copy_from_slice(&c.value.to_le_bytes());
        b[o + 8..o + 10].copy_from_slice(&c.unit.to_le_bytes());
        b[o + 10] = c.exp as u8;
        b[o + 11] = c.reserved;
    }
    let mut w = [0u64; SAMPLE_WORDS];
    pack(&b, &mut w);
    w
}

/// `None` unless exactly [`SAMPLE_WORDS`] words arrive. The content is not
/// validated here: [`Sample::channels`] bounds-checks the channel count, and
/// unknown registry values decode to `None` on access.
pub fn sample_from_words(w: &[u64]) -> Option<Sample> {
    if w.len() != SAMPLE_WORDS {
        return None;
    }
    let mut b = [0u8; SAMPLE_SIZE];
    unpack(w, &mut b);
    let u16_at = |o: usize| u16::from_le_bytes([b[o], b[o + 1]]);
    let u32_at = |o: usize| u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
    let mut ch = [Channel::default(); MAX_CHANNELS];
    for (c, o) in ch.iter_mut().zip((32..).step_by(12)) {
        *c = Channel {
            raw: u32_at(o) as i32,
            value: u32_at(o + 4) as i32,
            unit: u16_at(o + 8),
            exp: b[o + 10] as i8,
            reserved: b[o + 11],
        };
    }
    Some(Sample {
        timestamp: i64::from_le_bytes(b[0..8].try_into().unwrap()),
        sensor_id: u32_at(8),
        seq: u32_at(12),
        epoch: u32_at(16),
        kind: u16_at(20),
        flags: u16_at(22),
        ts_source: b[24],
        nchannels: b[25],
        reserved0: u16_at(26),
        reserved1: u32_at(28),
        ch,
    })
}

/// One I2C transfer as a server receives it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct I2cRequest {
    pub addr: u8,
    pub read_len: usize,
    pub deadline: Instant,
    write: [u8; MAX_WRITE],
    write_len: usize,
}

impl I2cRequest {
    pub fn write(&self) -> &[u8] {
        &self.write[..self.write_len]
    }
}

/// Encodes a request into `out` and returns the number of words used.
/// [`BusError::Invalid`] if either length is over its limit.
pub fn request_to_words(
    addr: u8,
    write: &[u8],
    read_len: usize,
    deadline: Instant,
    out: &mut [u64; REQUEST_WORDS],
) -> Result<usize, BusError> {
    if write.len() > MAX_WRITE || read_len > MAX_READ {
        return Err(BusError::Invalid);
    }
    out[0] = u64::from(addr) | (write.len() as u64) << 8 | (read_len as u64) << 16;
    out[1] = deadline.as_nanos() as u64;
    Ok(2 + pack(write, &mut out[2..]))
}

/// [`BusError::Invalid`] unless the header is well formed, both lengths are
/// within their limits, and exactly the words the header promises arrived.
/// The address is passed on as sent; the bus checks its range.
pub fn request_from_words(w: &[u64]) -> Result<I2cRequest, BusError> {
    let (&header, rest) = w.split_first().ok_or(BusError::Invalid)?;
    let (&deadline, data) = rest.split_first().ok_or(BusError::Invalid)?;
    let write_len = (header >> 8 & 0xff) as usize;
    let read_len = (header >> 16 & 0xff) as usize;
    if header >> 24 != 0
        || write_len > MAX_WRITE
        || read_len > MAX_READ
        || data.len() != words_for(write_len)
    {
        return Err(BusError::Invalid);
    }
    let mut write = [0u8; MAX_WRITE];
    unpack(data, &mut write[..write_len]);
    Ok(I2cRequest {
        addr: header as u8,
        read_len,
        deadline: Instant::from_nanos(deadline as i64),
        write,
        write_len,
    })
}

/// A transfer's outcome as a message label: 0 for success, otherwise the
/// error's code.
pub fn status_label(r: Result<(), BusError>) -> u64 {
    match r {
        Ok(()) => 0,
        Err(e) => e as u64,
    }
}

/// The inverse of [`status_label`]. A label no error has reads as
/// [`BusError::Io`].
pub fn status_from_label(label: u64) -> Result<(), BusError> {
    Err(match label {
        0 => return Ok(()),
        1 => BusError::Timeout,
        2 => BusError::Nak,
        3 => BusError::NoPresence,
        4 => BusError::Io,
        5 => BusError::Invalid,
        6 => BusError::Overflow,
        _ => BusError::Io,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Sample {
        let mut ch = [Channel::default(); MAX_CHANNELS];
        for (i, c) in ch.iter_mut().enumerate() {
            *c = Channel {
                raw: -1000 * i as i32 - 7,
                value: 1_000_000 * i as i32 + 3,
                unit: 0x100 + i as u16,
                exp: -(i as i8),
                reserved: 0x80 | i as u8,
            };
        }
        Sample {
            timestamp: -0x0123_4567_89ab_cdef,
            sensor_id: 0xdead_beef,
            seq: 42,
            epoch: 3,
            kind: 0x1234,
            flags: 0x8001,
            ts_source: 2,
            nchannels: 8,
            reserved0: 0xa55a,
            reserved1: 0x0f0f_f0f0,
            ch,
        }
    }

    #[test]
    fn samples_round_trip_every_field() {
        let s = sample();
        assert_eq!(sample_from_words(&sample_to_words(&s)), Some(s));
        assert_eq!(
            sample_from_words(&sample_to_words(&Sample::default())),
            Some(Sample::default())
        );
    }

    #[test]
    fn sample_fields_sit_at_their_layout_offsets() {
        let w = sample_to_words(&sample());
        let mut b = [0u8; SAMPLE_SIZE];
        unpack(&w, &mut b);
        assert_eq!(b[24], 2, "ts_source at 24");
        assert_eq!(b[25], 8, "nchannels at 25");
        // Channel 1 starts at 32 + 12: raw -1007.
        assert_eq!(&b[44..48], &(-1007i32).to_le_bytes());
        assert_eq!(b[44 + 10], (-1i8) as u8, "exp");
    }

    #[test]
    fn a_sample_of_the_wrong_length_is_refused() {
        let w = sample_to_words(&sample());
        assert_eq!(sample_from_words(&w[..15]), None);
        assert_eq!(sample_from_words(&[0; 17]), None);
    }

    #[test]
    fn requests_round_trip() {
        let mut out = [0; REQUEST_WORDS];
        for write in [
            &[][..],
            &[0xF7],
            &[1, 2, 3, 4, 5, 6, 7, 8, 9],
            &[0x55; MAX_WRITE],
        ] {
            let n =
                request_to_words(0x76, write, 24, Instant::from_nanos(123_456), &mut out).unwrap();
            assert_eq!(n, 2 + words_for(write.len()));
            let r = request_from_words(&out[..n]).unwrap();
            assert_eq!((r.addr, r.write(), r.read_len), (0x76, write, 24));
            assert_eq!(r.deadline, Instant::from_nanos(123_456));
        }
    }

    #[test]
    fn oversized_or_malformed_requests_are_invalid() {
        let mut out = [0; REQUEST_WORDS];
        let d = Instant::from_nanos(0);
        assert_eq!(
            request_to_words(0x76, &[0; MAX_WRITE + 1], 0, d, &mut out),
            Err(BusError::Invalid)
        );
        assert_eq!(
            request_to_words(0x76, &[], MAX_READ + 1, d, &mut out),
            Err(BusError::Invalid)
        );

        let n = request_to_words(0x76, &[1, 2, 3], 1, d, &mut out).unwrap();
        // Fewer or more words than the header promises.
        assert_eq!(request_from_words(&out[..n - 1]), Err(BusError::Invalid));
        let mut long = out;
        long[n] = 0;
        assert_eq!(request_from_words(&long[..n + 1]), Err(BusError::Invalid));
        // Lengths over the limits, and stray header bits.
        for header in [0x76 | 17 << 8, 0x76 | 33 << 16, 0x76 | 1 << 24] {
            assert_eq!(
                request_from_words(&[header, 0, 0, 0]),
                Err(BusError::Invalid)
            );
        }
        assert_eq!(request_from_words(&[]), Err(BusError::Invalid));
        assert_eq!(request_from_words(&[0x76]), Err(BusError::Invalid));
    }

    #[test]
    fn status_labels_round_trip_and_unknown_ones_are_io() {
        for e in [
            BusError::Timeout,
            BusError::Nak,
            BusError::NoPresence,
            BusError::Io,
            BusError::Invalid,
            BusError::Overflow,
        ] {
            assert_eq!(status_from_label(status_label(Err(e))), Err(e));
        }
        assert_eq!(status_from_label(status_label(Ok(()))), Ok(()));
        assert_eq!(status_from_label(99), Err(BusError::Io));
    }

    #[test]
    fn bytes_round_trip_through_words() {
        for n in 0..=MAX_READ {
            let bytes: Vec<u8> = (0..n as u8).map(|i| i.wrapping_mul(37)).collect();
            let mut w = [0u64; REPLY_WORDS];
            assert_eq!(pack(&bytes, &mut w), words_for(n));
            let mut back = vec![0; n];
            unpack(&w, &mut back);
            assert_eq!(back, bytes);
        }
    }
}
