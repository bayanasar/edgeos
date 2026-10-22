# sensor/sel4

The sensor stack on seL4: an I2C controller driver, the BMP280 driver and a
consumer, each in a Microkit protection domain of its own, for the Advantech
RSB-3720 (NXP i.MX 8M Plus).

Status: runs on the board; with no sensor connected, all 5 requests end in the bus's NAK, none in a timeout · not yet run with a sensor connected · Updated: 2026-10-21

## Three domains

| Domain | Priority | Holds | Calls |
|---|---|---|---|
| `i2c_imx` | 254 | I2C4's register page at `0x30a50000`, uncached | nothing |
| `bmp280` | 200 | no device | `i2c_imx`, once per transfer |
| `consumer` | 100 | nothing | `bmp280`, once per sample |

- `i2c_imx` runs `sensor-imx-i2c` on the mapped page. It cannot reach pin
  multiplexing, clocks or any other device. It polls, at the highest
  priority, so it caps each transfer at 100 ms, whatever deadline the client
  asks for.
- `bmp280` runs `sensor-bmp280` unchanged, over an `I2c` that calls
  `i2c_imx`. The bytes are copied both ways in message registers
  (`sensor-wire`): at most 16 written and 32 read per transfer.
- `consumer` asks for 5 samples, 500 ms apart, prints each, then a summary:
  `SENSOR_RESULT: samples=... nak=... timeout=... other=... of 5`.

A sample is 128 bytes, 16 message words, and is copied, not lent: for
buffers this small a copy costs less than granting and revoking a frame.

## Build

```sh
bash build.sh sdk /tmp/sensor-work     # Microkit SDK with an rsb3720 board
bash build.sh image /tmp/sensor-work   # /tmp/sensor-work/out/sensor.img
```

The SDK step clones Microkit 2.3.0 and seL4 16.0.0, applies
`microkit-rsb3720.patch` and builds in the pinned seL4 build image
(`../../kernel_architecture/sel4/pins.sh`). The patch adds an `rsb3720`
board: the `imx8mp-evk` kernel platform with the RSB-3720 overlay
(`kernel_architecture/sel4/boards/rsb3720.dts`: the firmware's memory
reservations and the UART3 console), user access to the generic timer's
virtual counter, and the loader's console on UART3 instead of the EVK's
UART2. Rust is rust-sel4 at the revision in `Cargo.toml`, built with the
nightly in `rust-toolchain.toml`; both steps install it into the work
directory, not on the host.

The portable crates build and test on stable from `..`:
`cargo test -p sensor-imx-i2c -p sensor-wire`. The controller driver is
tested against a register model of the controller with the fake bus's
devices behind it, including the BMP280 driver end to end.

## Run on the RSB-3720

The board's U-Boot leaves I2C4 clocked from the 24 MHz oscillator, but its
pads (ECSPI2_MISO and ECSPI2_SS0) muxed as GPIO. Mux them to I2C4, with the
values the board's own Linux device tree uses, then load the image at the
loader's link address and start it:

```
u-boot=> fatload mmc 1:1 0x41000000 sensor.img
u-boot=> mw.l 0x303301f8 0x12; mw.l 0x303301fc 0x12     # ALT2: I2C4_SCL, I2C4_SDA; SION
u-boot=> mw.l 0x30330458 0x1c3; mw.l 0x3033045c 0x1c3   # pad control
u-boot=> mw.l 0x303305bc 4; mw.l 0x303305c0 4           # I2C4 input select
u-boot=> go 0x41000000
```

I2C4 reaches header UIO2: pin 16 SDA, 18 SCL, 19 3.3 V, 20 GND (RSB-3720
user manual, ed. 1, section 2.4.2.20).

## Result with no sensor connected

The loader, the kernel and all three domains start. Each of the 5 requests
fails with a NAK (`0x102`, a bus error) after 171 to 184 us: about one
address byte at 100 kHz plus the controller's 50 us settling time. That
shows the controller's sequence and the calls between domains working on
the chip. It does not show the pads: an empty bus reads as a NAK whether or
not they are muxed. A sensor on the bus will.

## Not yet covered

- A sensor on the bus.
- Interrupt-driven transfers: the controller driver polls.
- Pins and clocks set by a component of the system rather than the boot
  loader.
- A release build of the kernel and domains.
