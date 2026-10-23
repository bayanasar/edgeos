// SPDX-License-Identifier: BSD-3-Clause
//! The BMP280 driver in a protection domain with no device access at all.
//! It reaches the sensor through the I2C controller's domain and hands each
//! sample to its consumer by copy, in message registers.

#![no_std]
#![no_main]

use pd_common::{
    BMP280_FROM_CONSUMER, BMP280_TO_I2C, CounterClock, ERR_REQUEST, I2cClient, OP_I2C_REPORT,
    OP_READ_SAMPLE, OP_REPORT, sample_status,
};
use sel4_microkit::{
    Channel, Handler, Infallible, MessageInfo, debug_println, protection_domain, with_msg_regs_mut,
};
use sensor_bmp280::{ADDR_SDO_LOW, Bmp280};
use sensor_wire::{SAMPLE_WORDS, sample_to_words};

/// The sensor's ID in its samples.
const SENSOR_ID: u32 = 1;

#[protection_domain]
fn init() -> HandlerImpl {
    debug_println!(
        "bmp280: expects a BMP280 at {:#04x}, no device mapped",
        ADDR_SDO_LOW
    );
    HandlerImpl {
        sensor: Bmp280::new(I2cClient::new(BMP280_TO_I2C), ADDR_SDO_LOW, SENSOR_ID, 1),
        clock: CounterClock::new(),
        ready: false,
    }
}

struct HandlerImpl {
    sensor: Bmp280<I2cClient>,
    clock: CounterClock,
    ready: bool,
}

impl Handler for HandlerImpl {
    type Error = Infallible;

    fn protected(
        &mut self,
        channel: Channel,
        msg_info: MessageInfo,
    ) -> Result<MessageInfo, Self::Error> {
        if channel != BMP280_FROM_CONSUMER {
            return Ok(MessageInfo::new(ERR_REQUEST, 0));
        }
        match msg_info.label() {
            OP_READ_SAMPLE => {}
            OP_REPORT => {
                let reply = BMP280_TO_I2C.pp_call(MessageInfo::new(OP_I2C_REPORT, 0));
                return Ok(MessageInfo::new(reply.label(), 0));
            }
            _ => return Ok(MessageInfo::new(ERR_REQUEST, 0)),
        }
        // Initialise on demand, so a sensor connected later is found by the
        // next request.
        if !self.ready {
            if let Err(e) = self.sensor.init(&self.clock) {
                return Ok(MessageInfo::new(sample_status(Some(e)), 0));
            }
            self.ready = true;
        }
        match self.sensor.read(&self.clock) {
            Ok(sample) => {
                let words = sample_to_words(&sample);
                with_msg_regs_mut(|mrs| mrs[..SAMPLE_WORDS].copy_from_slice(&words));
                Ok(MessageInfo::new(0, SAMPLE_WORDS))
            }
            Err(e) => {
                self.ready = false;
                Ok(MessageInfo::new(sample_status(Some(e)), 0))
            }
        }
    }
}
