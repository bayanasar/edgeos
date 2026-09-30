// SPDX-License-Identifier: BSD-3-Clause
//! Bus primitives a sensor driver may ask of its platform, and the
//! fixed-layout records it hands upward.
//!
//! A driver is generic over the traits in [`bus`]: no platform calls, no heap,
//! no floating point, no global state. A transport (Linux, a seL4 component,
//! or a fake bus in tests) implements the traits it can provide. A driver that
//! needs a primitive the transport lacks does not compile against it.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

pub mod bus;
pub mod sample;

pub use bus::{BusError, Clock, Gpio, I2c, Instant, OneWire, Spi};
pub use sample::{Channel, Event, Sample};
