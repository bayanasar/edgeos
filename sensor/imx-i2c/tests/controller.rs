// SPDX-License-Identifier: BSD-3-Clause
//! The driver against a register model of the controller, with the fake
//! bus's device models behind it.

use std::cell::Cell;

use sensor_bmp280::{ADDR_SDO_LOW, Bmp280, CHIP_ID};
use sensor_core::bus::{BusError, Clock, I2c, Instant};
use sensor_fake::{FakeClock, FakeDevice, FakeI2c, RegisterDevice, Transfer};
use sensor_imx_i2c::*;

/// A clock that moves 1 us each time it is read, so polling loops reach
/// their deadlines.
#[derive(Default)]
struct StepClock(Cell<i64>);

impl Clock for StepClock {
    fn now(&self) -> Instant {
        self.0.set(self.0.get() + 1_000);
        Instant::from_nanos(self.0.get())
    }

    fn sleep_until(&self, t: Instant) {
        if t.as_nanos() > self.0.get() {
            self.0.set(t.as_nanos());
        }
    }
}

#[derive(Debug, PartialEq)]
enum Phase {
    Idle,
    /// After a start or repeated start: the next byte is an address.
    Address,
    Writing,
    /// The address was refused; the master must stop.
    Refused,
    /// Receiving from a device; `data` is what it returns.
    Reading {
        data: Vec<u8>,
        clocked: usize,
    },
}

/// The controller's registers and the bus behind them. Bytes written to a
/// device are held until the transfer reads or stops, then handed to the
/// device model in one call, as the fake bus does.
struct Model {
    devices: Vec<Box<dyn FakeDevice>>,
    control: u8,
    ifdr: u8,
    iif: bool,
    ial: bool,
    rxak: bool,
    busy: bool,
    data: u8,
    phase: Phase,
    addr: u8,
    written: Vec<u8>,
    /// Whether the master acknowledged each byte it received in this read.
    acks: Vec<bool>,
    log: Vec<Transfer>,
    violations: Vec<String>,
    /// Never finish a byte.
    hang: bool,
    /// Another master holds the bus.
    held: bool,
}

impl Model {
    fn new(devices: Vec<Box<dyn FakeDevice>>) -> Self {
        Model {
            devices,
            control: 0,
            ifdr: 0,
            iif: false,
            ial: false,
            rxak: false,
            busy: false,
            data: 0,
            phase: Phase::Idle,
            addr: 0,
            written: Vec::new(),
            acks: Vec::new(),
            log: Vec::new(),
            violations: Vec::new(),
            hang: false,
            held: false,
        }
    }

    fn with(dev: impl FakeDevice + 'static) -> Self {
        Self::new(vec![Box::new(dev)])
    }

    fn violation(&mut self, what: &str) {
        self.violations.push(format!("{what} in {:?}", self.phase));
    }

    fn byte_done(&mut self) {
        if !self.hang {
            self.iif = true;
        }
    }

    fn device(&mut self, addr: u8) -> Option<&mut Box<dyn FakeDevice>> {
        self.devices.iter_mut().find(|d| d.addr() == addr)
    }

    /// Ends the transfer in progress at a stop.
    fn finish(&mut self) {
        match std::mem::replace(&mut self.phase, Phase::Idle) {
            Phase::Writing => {
                let (addr, written) = (self.addr, std::mem::take(&mut self.written));
                self.device(addr)
                    .unwrap()
                    .transfer(&written, &mut [])
                    .unwrap();
                self.log.push(Transfer {
                    addr,
                    write: written,
                    read_len: 0,
                });
            }
            Phase::Reading { clocked, .. } => {
                let acks = std::mem::take(&mut self.acks);
                if acks.last() != Some(&false) || acks[..acks.len() - 1].contains(&false) {
                    self.violations
                        .push(format!("only the last byte read may be refused: {acks:?}"));
                }
                self.log.push(Transfer {
                    addr: self.addr,
                    write: std::mem::take(&mut self.written),
                    read_len: clocked,
                });
            }
            Phase::Address if !self.hang => self.violations.push("stop right after a start".into()),
            Phase::Address => {}
            Phase::Idle | Phase::Refused => {}
        }
        self.written.clear();
    }

    fn write_control(&mut self, value: u8) {
        let old = self.control;
        self.control = value & !I2CR_RSTA;
        if value & I2CR_IEN == 0 {
            if old & I2CR_MSTA != 0 {
                self.violation("disabled while owning the bus");
            }
            self.busy = self.held;
            self.iif = false;
            self.phase = Phase::Idle;
            return;
        }
        if value & I2CR_IIEN != 0 {
            self.violation("interrupt enabled in a polled driver");
        }
        let (was, is) = (old & I2CR_MSTA != 0, value & I2CR_MSTA != 0);
        if !was && is {
            if self.busy {
                self.ial = true;
                self.control &= !I2CR_MSTA;
                return;
            }
            self.busy = true;
            self.phase = Phase::Address;
        } else if was && !is {
            self.finish();
            self.busy = false;
        } else if was && value & I2CR_RSTA != 0 {
            if self.phase != Phase::Writing {
                self.violation("repeated start");
            }
            self.phase = Phase::Address;
        }
    }

    fn write_data(&mut self, value: u8) {
        if self.control & I2CR_MTX == 0 {
            self.violation("data written in receive mode");
            return;
        }
        if self.hang {
            // The byte never completes, so nothing on the bus changes.
            return;
        }
        match self.phase {
            Phase::Address => {
                let (addr, read) = (value >> 1, value & 1 != 0);
                if self.device(addr).is_none() {
                    self.rxak = true;
                    self.phase = Phase::Refused;
                } else if !read {
                    self.rxak = false;
                    self.addr = addr;
                    self.phase = Phase::Writing;
                } else {
                    if !self.written.is_empty() && addr != self.addr {
                        self.violation("read from another device after a write");
                    }
                    self.rxak = false;
                    self.addr = addr;
                    let mut data = vec![0; 64];
                    let written = self.written.clone();
                    self.device(addr)
                        .unwrap()
                        .transfer(&written, &mut data)
                        .unwrap();
                    self.phase = Phase::Reading { data, clocked: 0 };
                }
                self.byte_done();
            }
            Phase::Writing => {
                self.written.push(value);
                self.rxak = false;
                self.byte_done();
            }
            _ => self.violation("byte written"),
        }
    }

    fn read_data(&mut self) -> u8 {
        let value = self.data;
        let receiving = self.control & (I2CR_MSTA | I2CR_MTX) == I2CR_MSTA;
        let ack = self.control & I2CR_TXAK == 0;
        if let Phase::Reading { data, clocked } = &mut self.phase {
            if receiving {
                self.data = data[*clocked];
                *clocked += 1;
                self.acks.push(ack);
                self.byte_done();
            }
        }
        value
    }
}

impl Regs for Model {
    fn read(&mut self, offset: usize) -> u8 {
        match offset {
            IFDR => self.ifdr,
            I2CR => self.control,
            I2SR => {
                (if self.busy { I2SR_IBB } else { 0 })
                    | (if self.ial { I2SR_IAL } else { 0 })
                    | (if self.iif { I2SR_IIF } else { 0 })
                    | (if self.rxak { I2SR_RXAK } else { 0 })
            }
            I2DR => self.read_data(),
            _ => panic!("read of offset {offset:#x}"),
        }
    }

    fn write(&mut self, offset: usize, value: u8) {
        match offset {
            IFDR => self.ifdr = value,
            I2CR => self.write_control(value),
            I2SR => {
                // Written as 0, IIF and IAL clear.
                self.iif &= value & I2SR_IIF != 0;
                self.ial &= value & I2SR_IAL != 0;
            }
            I2DR => self.write_data(value),
            _ => panic!("write of offset {offset:#x}"),
        }
    }
}

fn deadline() -> Instant {
    Instant::from_nanos(1_000_000_000)
}

fn counter(addr: u8) -> RegisterDevice {
    let mut dev = RegisterDevice::new(addr);
    for (i, r) in dev.regs.iter_mut().enumerate() {
        *r = i as u8 ^ 0xa5;
    }
    dev
}

/// Calibration and data registers of a SunFounder BMP280 module, as in the
/// BMP280 driver's own tests.
const MODULE_CALIB: [u8; 24] = [
    0x90, 0x6d, 0x6e, 0x63, 0x32, 0x00, 0x4d, 0x90, 0xc0, 0xd5, 0xd0, 0x0b, 0x31, 0x1b, 0xc8, 0xff,
    0xf9, 0xff, 0x8c, 0x3c, 0xf8, 0xc6, 0x70, 0x17,
];
const MODULE_DATA: [u8; 6] = [0x55, 0x16, 0x00, 0x86, 0x8c, 0x00];

/// A BMP280 whose forced conversions finish at once.
fn bmp280() -> RegisterDevice {
    let mut dev = RegisterDevice::new(ADDR_SDO_LOW).with_hook(Box::new(|reg, v, regs| {
        if reg == 0xE0 && v == 0xB6 {
            regs[0xF4] = 0;
        }
        if reg == 0xF4 && v & 0b11 == 0b01 {
            regs[0xF7..0xF7 + 6].copy_from_slice(&MODULE_DATA);
        }
    }));
    dev.regs[0xD0] = CHIP_ID;
    dev.regs[0x88..0x88 + 24].copy_from_slice(&MODULE_CALIB);
    dev
}

fn assert_released(model: &Model) {
    assert!(model.violations.is_empty(), "{:?}", model.violations);
    assert_eq!(model.control, 0, "the block is left disabled");
    assert_eq!(model.busy, model.held, "the bus is released");
}

#[test]
fn bmp280_through_the_controller_matches_the_fake_bus() {
    let mut model = Model::with(bmp280());
    let (clock_a, clock_b) = (FakeClock::new(), FakeClock::new());

    let mut via_controller = Bmp280::new(
        ImxI2c::new(&mut model, StepClock::default(), IFDR_24MHZ_100KHZ),
        ADDR_SDO_LOW,
        7,
        1,
    );
    let mut via_fake = Bmp280::new(FakeI2c::with_device(bmp280()), ADDR_SDO_LOW, 7, 1);
    via_controller.init(&clock_a).unwrap();
    via_fake.init(&clock_b).unwrap();
    for _ in 0..3 {
        let a = via_controller.read(&clock_a).unwrap();
        let b = via_fake.read(&clock_b).unwrap();
        assert_eq!(a, b);
        assert_eq!(a.channels().unwrap().len(), 2);
    }
    let _ = via_controller.release();
    let fake = via_fake.release();

    assert_eq!(model.log, fake.log, "the same transfers reached the chip");
    assert_eq!(model.ifdr, IFDR_24MHZ_100KHZ);
    assert_released(&model);
}

#[test]
fn reads_of_every_length_acknowledge_all_but_the_last_byte() {
    for n in [1, 2, 3, 6, 24] {
        let mut model = Model::with(counter(0x50));
        let mut buf = vec![0; n];
        let mut i2c = ImxI2c::new(&mut model, StepClock::default(), IFDR_24MHZ_100KHZ);
        i2c.transfer(0x50, &[0x10], &mut buf, deadline()).unwrap();
        let _ = i2c.release();
        let want: Vec<u8> = (0x10..0x10 + n).map(|i| i as u8 ^ 0xa5).collect();
        assert_eq!(buf, want, "{n} bytes");
        assert_eq!(model.log.len(), 1);
        assert_eq!(model.log[0].read_len, n, "exactly {n} bytes clocked in");
        assert_released(&model);
    }
}

#[test]
fn read_without_a_register_write_has_no_repeated_start() {
    let mut model = Model::with(counter(0x50));
    let mut buf = [0; 2];
    let mut i2c = ImxI2c::new(&mut model, StepClock::default(), IFDR_24MHZ_100KHZ);
    i2c.transfer(0x50, &[], &mut buf, deadline()).unwrap();
    let _ = i2c.release();
    assert_eq!(buf, [0xa5, 0xa4]);
    assert_eq!(model.log[0].write, Vec::<u8>::new());
    assert_released(&model);
}

#[test]
fn write_only_reaches_the_device_at_the_stop() {
    let mut model = Model::with(counter(0x50));
    let mut i2c = ImxI2c::new(&mut model, StepClock::default(), IFDR_24MHZ_100KHZ);
    i2c.transfer(0x50, &[0x20, 1, 2, 3], &mut [], deadline())
        .unwrap();
    let _ = i2c.release();
    assert_eq!(
        model.log,
        [Transfer {
            addr: 0x50,
            write: vec![0x20, 1, 2, 3],
            read_len: 0
        }]
    );
    assert_released(&model);
}

#[test]
fn an_absent_device_is_a_nak_and_the_bus_is_released() {
    for read_len in [0, 1, 4] {
        let mut model = Model::with(counter(0x50));
        let mut buf = vec![0; read_len];
        let write: &[u8] = if read_len == 1 { &[] } else { &[0x00] };
        let mut i2c = ImxI2c::new(&mut model, StepClock::default(), IFDR_24MHZ_100KHZ);
        assert_eq!(
            i2c.transfer(0x76, write, &mut buf, deadline()),
            Err(BusError::Nak)
        );
        // The bus works again at once.
        assert_eq!(i2c.transfer(0x50, &[0], &mut [0], deadline()), Ok(()));
        let _ = i2c.release();
        assert!(model.log.iter().all(|t| t.addr == 0x50));
        assert_released(&model);
    }
}

#[test]
fn a_byte_that_never_finishes_times_out_and_releases_the_bus() {
    let mut model = Model::with(counter(0x50));
    model.hang = true;
    let mut i2c = ImxI2c::new(&mut model, StepClock::default(), IFDR_24MHZ_100KHZ);
    let soon = Instant::from_nanos(2_000_000);
    assert_eq!(
        i2c.transfer(0x50, &[0], &mut [0], soon),
        Err(BusError::Timeout)
    );
    let _ = i2c.release();
    assert!(model.log.is_empty());
    assert_released(&model);
}

#[test]
fn a_bus_held_by_another_master_is_an_io_error() {
    let mut model = Model::with(counter(0x50));
    model.held = true;
    model.busy = true;
    let mut i2c = ImxI2c::new(&mut model, StepClock::default(), IFDR_24MHZ_100KHZ);
    assert_eq!(
        i2c.transfer(0x50, &[0], &mut [], deadline()),
        Err(BusError::Io)
    );
    let _ = i2c.release();
    assert!(model.log.is_empty());
    assert_released(&model);
}

#[test]
fn bad_arguments_touch_no_register() {
    let mut model = Model::with(counter(0x50));
    let mut i2c = ImxI2c::new(&mut model, StepClock::default(), IFDR_24MHZ_100KHZ);
    assert_eq!(
        i2c.transfer(0x80, &[0], &mut [], deadline()),
        Err(BusError::Invalid)
    );
    assert_eq!(
        i2c.transfer(0x50, &[], &mut [], deadline()),
        Err(BusError::Invalid)
    );
    let _ = i2c.release();
    assert_eq!(model.ifdr, 0);
    assert!(model.log.is_empty());
}
