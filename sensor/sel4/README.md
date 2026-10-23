# sensor/sel4

The sensor stack on seL4: a timer, an I2C controller driver, the BMP280
driver and a consumer, each in a Microkit protection domain of its own, for
the Advantech RSB-3720 (NXP i.MX 8M Plus).

Status: interrupt-driven, with a timer domain for deadlines; passes in QEMU's imx8mp-evk (`qemu.sh`) and on the board with no sensor connected · Updated: 2026-10-22

## Four domains

| Domain | Priority | Holds | Calls |
|---|---|---|---|
| `timer` | 254 | GPT1's register page at `0x302d0000`, uncached, and its interrupt (SPI 55) | nothing |
| `i2c_imx` | 253 | I2C4's register page at `0x30a50000`, uncached, and its interrupt (SPI 38) | `timer`, to set and cancel a deadline |
| `bmp280` | 200 | no device | `i2c_imx`, once per transfer, and once to pass on the request for counts |
| `consumer` | 100 | nothing | `bmp280`, once per sample, and once for `i2c_imx`'s counts |

- `timer` runs `sensor-imx-gpt`. Its one client asks for a signal at a
  deadline on the shared counter clock; the domain arms the timer's compare
  and signals the client when it matches. It measures the timer's rate
  against the counter at start, spinning for 1 ms.
- `i2c_imx` runs `sensor-imx-i2c` on the mapped page, with an
  interrupt-driven waiter. It cannot reach pin multiplexing, clocks or any
  other device. While a byte is on the bus it blocks until the controller
  interrupts or the timer signals the deadline, so lower priorities run in
  the meantime. What still spins, per transfer: 50 us after enabling the
  controller, and at most three waits for the bus to turn busy or idle
  (after the start, after a repeated start, and for the stop), each at most
  1 ms and normally about one bit time. So a transfer keeps lower
  priorities waiting about 3 ms at worst and normally some tens of
  microseconds, where the polled version could keep them waiting the whole
  transfer. A transfer holds the bus for at most 100 ms, whatever deadline
  the client asks for. If the timer does not count, the domain polls
  instead, and that 100 ms is again how long lower priorities may wait.
- `i2c_imx` counts how its waits end: woken by the controller's interrupt,
  woken by the timer, or returned to poll because the timer could not help.
  A byte already complete when the driver first looks is not waited for and
  not counted, so `irq` above 0 is what shows the interrupt path was taken.
- `i2c_imx` now depends on `timer` to end every wait: if GPT1 stopped
  counting after its rate was measured, or its interrupt were lost, a wait
  would never end, with no fault and nothing logged, and `bmp280` and
  `consumer` would stay blocked in their calls. The polled version always
  ended in a timeout.
- Inside a protected procedure `i2c_imx` cannot return to Microkit's event
  loop, so it blocks with `seL4_Wait` on its own endpoint, which keeps the
  reply owed to its caller and still receives its notifications. That is
  sound only with one client, blocked in the call being served: a second
  client calling in would find no reply object. More clients need a bus
  multiplexer in front of the domain.
- `bmp280` runs `sensor-bmp280` unchanged, over an `I2c` that calls
  `i2c_imx`. The bytes are copied both ways in message registers
  (`sensor-wire`): at most 16 written and 32 read per transfer.
- `consumer` asks for 5 samples, 500 ms apart, and prints each with how
  long it took. Then it asks, through `bmp280`, for `i2c_imx`'s counts,
  which `i2c_imx` prints as
  `I2C_WAITS: transfers=... irq=... timer=... polled=...`, and prints a
  summary: `SENSOR_RESULT: samples=... nak=... timeout=... other=... of 5`.
  The counts wait until the end because a line at 115200 baud takes about
  4 ms to print, which would swamp the timing of a transfer.

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
`cargo test -p sensor-imx-i2c -p sensor-imx-gpt -p sensor-wire`. The
controller driver is tested against a register model of the controller with
the fake bus's devices behind it, including the BMP280 driver end to end,
polled and interrupt-driven; the timer driver against a register model of
the timer.

## Run in QEMU

```sh
bash qemu.sh /tmp/sensor-work          # after build.sh image
```

Runs the image on QEMU's `imx8mp-evk` twice, with I2C4 empty and with a
BMP280's registers on it, and checks the consumer's summary and the
controller domain's wait counts. `qemu.sh`
says where QEMU's machine differs from the board: it needs a stub in place
of the secure firmware's interrupt setup, its I2C controller completes
bytes at once and does not flag a refused address, and its timer runs at a
different rate. So QEMU checks the domains, the calls between them, the
copy of every byte and the timer domain's timeouts; it cannot check the
controller domain being woken by its own interrupt.

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

The timer domain also needs GPT1 clocked, and U-Boot leaves it so: on the
board, before `go`, its root clock and gate read as below (offsets from
Linux's `clk-imx8mp.c`). If either reads otherwise, the `mw.l` line sets
them, with the root on the 24 MHz oscillator:

```
u-boot=> md.l 0x3038b580 1      # GPT1 root: 0x10000000, enabled, 24 MHz oscillator
u-boot=> md.l 0x30384100 1      # GPT1 gate: 0x00000003, on
u-boot=> mw.l 0x3038b580 0x10000000; mw.l 0x30384100 3
```

I2C4 reaches header UIO2: pin 16 SDA, 18 SCL, 19 3.3 V, 20 GND (RSB-3720
user manual, ed. 1, section 2.4.2.20).

## Result in QEMU (2026-10-22)

With I2C4 empty, each of the 5 requests ends in a timeout (`0x101`) after
50 to 51 ms, the BMP280 driver's deadline, signalled by the timer domain.
With a BMP280's registers on the bus, all 5 samples arrive with the values
the host tests compute from the same registers: 100526.390 Pa and
31.06 C.

## Result on the board with no sensor connected (2026-10-22)

Three runs of the same image. In each, the timer domain measures GPT1 at
2993877 to 2994876 Hz, the 24 MHz oscillator divided by 8, and each of the
5 requests fails with a NAK (`0x102`) after 175 to 189 us, as with the
polled version below (171 to 184 us). Each run ends with
`I2C_WAITS: transfers=5 irq=5 timer=0 polled=0`: every transfer waited for
its address byte and was woken by the controller's interrupt, which the
chip raises for a refused address. No wait was ended by the timer or fell
back to polling. So on the board the controller domain blocks on its own
interrupt, which QEMU never makes it do.

## Result on the board with no sensor connected (polled version, 2026-10-21)

The loader, the kernel and all three domains start. Each of the 5 requests
fails with a NAK (`0x102`, a bus error) after 171 to 184 us: about one
address byte at 100 kHz plus the controller's 50 us settling time. That
shows the controller's sequence and the calls between domains working on
the chip. It does not show the pads: an empty bus reads as a NAK whether or
not they are muxed. A sensor on the bus will.

## Not yet covered

- A sensor on the bus.
- A transfer ended by the timer's deadline on the board; QEMU covers it.
- Pins and clocks set by a component of the system rather than the boot
  loader.
- More than one client of the controller or the timer.
- The BMP280 domain's wait for a conversion, which still spins.
- A release build of the kernel and domains.
