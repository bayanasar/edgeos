# sensor

Portable sensor drivers in Rust, written once and run on Linux, as isolated
seL4 components, and against a fake bus in tests.

Status: interface only. No drivers yet.

## `sensor-core`

The contract every driver and transport shares. `no_std`, no heap, no floating
point, no `unsafe`.

- **Bus primitives** (`bus`): `Clock`, `Gpio` (levels and timestamped edges),
  `I2c`, `OneWire` and `Spi`. A transport implements the ones it provides; a
  driver that needs a primitive the transport lacks does not compile against
  it. Every blocking operation takes an absolute deadline on the monotonic
  clock, and a driver never retries on its own.
- **Records** (`sample`): `Sample` (128 bytes, up to 8 channels) and `Event`
  (32 bytes). Fixed layout, no pointers, no padding, integer values scaled by
  a decimal exponent. Each record carries where its timestamp came from, a
  sequence number and an epoch that changes when a driver restarts.
  Consumers do not trust the producer: `Sample::channels` refuses a channel
  count above 8, and unknown registry values decode to `None`.

There is no ADC primitive (an ADC is a device on a bus) and no pulse primitive
(a pulse is two timestamped edges).

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
