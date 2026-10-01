# sensor

Portable sensor drivers in Rust, written once and run on Linux, as isolated
seL4 components, and against a fake bus in tests.

Status: three drivers (BMP280, PCF8591, DS18B20), verified on a Raspberry Pi 5 over Linux.

| Crate | What it is |
|---|---|
| `core` (`sensor-core`) | The contract: bus traits and sample records |
| `drivers/bmp280` (`sensor-bmp280`) | Bosch BMP280 pressure and temperature, I2C |
| `drivers/pcf8591` (`sensor-pcf8591`) | NXP PCF8591 four-input 8-bit ADC, I2C |
| `drivers/ds18b20` (`sensor-ds18b20`) | Maxim DS18B20 thermometer, 1-Wire |
| `fake` (`sensor-fake`) | Fake clock, I2C bus with pluggable device models, 1-Wire bus |
| `linux` (`sensor-linux`) | Linux transport (`/dev/i2c-N`, w1 netlink, `CLOCK_MONOTONIC`) and the `bmp280-read`, `pcf8591-read`, `ds18b20-read` tools |

## `sensor-core`

The contract every driver and transport shares. `no_std`, no heap, no floating
point, no `unsafe`.

- **Bus primitives** (`bus`): `Clock`, `Gpio` (levels and timestamped edges),
  `I2c`, `OneWire` and `Spi`. A transport implements the ones it provides; a
  driver that needs a primitive the transport lacks does not compile against
  it. Every blocking operation takes an absolute deadline on the monotonic
  clock, and a driver never retries on its own. On Linux, I2C checks the
  deadline before each transfer and is otherwise bounded by the kernel
  adapter's own timeout; 1-Wire waits no longer than the deadline.
- **Records** (`sample`): `Sample` (128 bytes, up to 8 channels) and `Event`
  (32 bytes). Fixed layout, no pointers, no padding, integer values scaled by
  a decimal exponent. Each record carries where its timestamp came from, a
  sequence number and an epoch that changes when a driver restarts.
  Consumers do not trust the producer: `Sample::channels` refuses a channel
  count above 8, and unknown registry values decode to `None`.

There is no ADC primitive (an ADC is a device on a bus) and no pulse primitive
(a pulse is two timestamped edges).

## Running on a Raspberry Pi

`.cargo/config.toml` links the static `aarch64-unknown-linux-musl` target with
`rust-lld`, so no cross C toolchain is needed:

```sh
rustup target add aarch64-unknown-linux-musl
cargo build --release --target aarch64-unknown-linux-musl -p sensor-linux --bin bmp280-read
# copy target/aarch64-unknown-linux-musl/release/bmp280-read to the Pi, then:
./bmp280-read --bus 1 --addr 0x76 --count 10 --interval-ms 1000
```

The user needs access to `/dev/i2c-1` (the `i2c` group on Raspberry Pi OS),
and I2C must be enabled (`raspi-config nonint do_i2c 0`).

1-Wire goes through the kernel's w1 core (`raspi-config nonint do_onewire 0`
loads `w1-gpio` on GPIO 4). The transport sends raw reset, write and read
commands over the w1 netlink connector, so the DS18B20 protocol runs in the
driver, not in the kernel's `w1_therm`. On the tested kernel this needs no
root privileges. The DS18B20 must be externally powered (VCC connected);
parasite power needs a strong pull-up the transport cannot give. `ds18b20-read --bus N` takes the master number of
`w1_bus_masterN`.

## Checks

```sh
cargo test
cargo clippy --all-targets -- -D warnings
for t in x86_64-unknown-linux-gnu i686-unknown-linux-gnu aarch64-unknown-linux-gnu \
         armv7-unknown-linux-gnueabihf riscv64gc-unknown-none-elf thumbv7em-none-eabihf; do
    cargo build --release -p sensor-core --target "$t"
done
```

Record sizes, offsets and alignment are compile-time assertions, so the
cross-target builds are the layout test. The records need their explicit
8-byte alignment: without it, i686 aligns the 64-bit timestamp to 4 and the
assertions fail.
