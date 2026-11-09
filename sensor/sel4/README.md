# sensor/sel4

The sensor stack on seL4: a timer, an I2C controller driver, the BMP280
driver and a consumer, each in a Microkit protection domain of its own, for
the Advantech RSB-3720 (NXP i.MX 8M Plus).

Status: interrupt-driven, with a timer domain for deadlines and for the BMP280's conversions; samples a BMP280 on the board, and passes in QEMU's imx8mp-evk (`qemu.sh`) · Updated: 2026-11-08

## Four domains

| Domain | Priority | Holds | Calls |
|---|---|---|---|
| `timer` | 254 | GPT1's register page at `0x302d0000`, uncached, and its interrupt (SPI 55) | nothing |
| `i2c_imx` | 253 | I2C4's register page at `0x30a50000`, uncached, and its interrupt (SPI 38) | `timer`, to set and cancel a deadline |
| `bmp280` | 200 | no device | `i2c_imx`, once per transfer, and once to pass on the request for counts; `timer`, to set the end of each sleep |
| `consumer` | 100 | nothing | `bmp280`, once per sample, and once for the counts of `bmp280` and `i2c_imx` |

- `timer` runs `sensor-imx-gpt`. Each client has one of GPT1's three
  compares, `i2c_imx` the first and `bmp280` the second, and asks for a
  signal at a deadline on the shared counter clock; the domain arms that
  compare and signals that client when it matches. It never blocks in a
  call, so it can serve both. It measures the timer's rate against the
  counter at start, spinning for 1 ms, with each counter read timed from
  both sides so the read's own time does not skew the rate.
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
- `i2c_imx` counts how its waits end: woken by the controller's interrupt
  (`irq`), ended at the deadline (`expired`), or returned to poll because
  the timer could not help (`polled`). Apart from those it counts the
  timer's signals that the counter clock overrules (`early`): a signal that
  comes while the deadline is still ahead, a little early or left from a
  deadline already cancelled, after which the wait asks again. A byte
  already complete when the driver first looks is not waited for and not
  counted, so `irq` above 0 is what shows the interrupt path was taken.
- `i2c_imx` and `bmp280` now depend on `timer` to end every wait and every
  sleep: if GPT1 stopped counting after its rate was measured, or its
  interrupt were lost, a wait or a sleep would never end, with no fault and
  nothing logged, and the domains below would stay blocked in their calls.
  The polled version always ended in a timeout.
- Inside a protected procedure `i2c_imx` and `bmp280` cannot return to
  Microkit's event loop, so they block with `seL4_Wait` on their own
  endpoint (`wait_input` in `common`), which keeps the reply owed to the
  caller and still receives the domain's notifications. That is sound only
  with one client, blocked in the call being served: a second client
  calling in would find no reply object. More clients of `i2c_imx` need a
  bus multiplexer in front of it.
- `bmp280` runs `sensor-bmp280` unchanged, over an `I2c` that calls
  `i2c_imx`. The bytes are copied both ways in message registers
  (`sensor-wire`): at most 16 written and 32 read per transfer. Its
  sleeps, 2 ms after a reset, 5.5 ms for each conversion and 1 ms between
  status polls, block: each asks `timer` for a signal at its end and waits
  for it, so lower priorities run in the meantime. It counts how they end:
  woken by the timer's signal at the end (`timer`), signals the counter
  clock overrules (`early`, after which it asks again), and sleeps spun to
  the end because the timer could not help (`spun`).
- `consumer` asks for 5 samples, 500 ms apart, and prints each with how
  long it took. It spins between them: at the lowest priority it keeps no
  one else waiting. Then it asks `bmp280` for the counts, which `bmp280`
  prints as `BMP280_SLEEPS: timer=... early=... spun=...` and passes on to
  `i2c_imx`, which prints
  `I2C_WAITS: transfers=... irq=... expired=... early=... polled=...`; and
  it prints a summary:
  `SENSOR_RESULT: samples=... nak=... timeout=... other=... of 5`.
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
counts of the controller domain's waits and the BMP280 domain's sleeps.
`qemu.sh` says where QEMU's machine differs from the board: it needs a
stub in place of the secure firmware's interrupt setup, its I2C controller
completes bytes at once and does not flag a refused address, and its timer
runs at a different rate. So QEMU checks the domains, the calls between
them, the copy of every byte, the timer domain's timeouts and the BMP280
domain's sleeps on the timer's second compare; it cannot check the
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

## Result on the board with a BMP280 (2026-11-08)

A BMP280 module on UIO2: VCC to pin 19, GND to pin 20, SDI to pin 16 (SDA),
SCK to pin 18 (SCL), CS to 3.3 V, which selects I2C, and SDO to ground,
which sets address 0x76. From the board's Linux, `i2cdetect -y 3` finds it
at 0x76, and its chip ID register, 0xD0, reads 0x58.

The image built from the parent of this change (SHA-256 `a0e8848c...`)
ran twice, and this change's image (`004170e6...`) three times. In every
run all 5 requests return a sample, 101711 to 101737 Pa and 23.64 to
23.80 C. The first request takes 13.7 ms, the driver's initialisation
included; the others 7.31 to 7.32 ms, 5.5 ms of it the conversion. Every
run ends with `irq=121` and every other count 0: the 20 transfers put
121 bytes on the bus (41 to initialise, 16 per sample), and the
controller's interrupt woke the driver for each one.

GPT1 measures 2994876 Hz in both runs of the parent and 2997873 to
2998061 Hz in this change's, now that each counter read is timed from both
sides: 0.17 % below 3 MHz, and then 0.06 to 0.07 %. With this change every
run ends with `BMP280_SLEEPS: timer=6 early=0 spun=0`: the 2 ms start-up
after the reset and the 5 conversions each blocked until the timer's
signal on its second compare, with no early signal and no sleep spun. The
requests take as long as before; what changes is that the processor is
free while the BMP280 domain waits.

## Result in QEMU, with the BMP280 domain's sleeps on the timer (2026-11-08)

Three runs of the same image. The timer domain measures GPT1 at 15953611
to 15975230 Hz; before each counter read was timed from both sides it read
15.5 to 15.7 MHz. With I2C4 empty, each of the 5 requests again ends in a
timeout (`0x101`) after about 50 ms (50.4 to 51.2 ms in the last run), and
each run ends with
`BMP280_SLEEPS: timer=0 early=0 spun=0` and
`I2C_WAITS: transfers=5 irq=0 expired=5 early=0 polled=0`. With the rate
measured as before, the same bus showed `expired=5 early=5`: the timer's
signal came early once in every 50 ms wait, and the wait asked again.

With a BMP280's registers on the bus, all 5 samples arrive with the same
values as before (in the last run, the first request after 13 ms and the
others after 6.7 to 7.0 ms), and each run ends with
`BMP280_SLEEPS: timer=6 early=0 spun=0` and
`I2C_WAITS: transfers=20 irq=0 expired=0 early=0 polled=0`: the 2 ms
start-up and the 5 conversions of 5.5 ms each slept until the timer's
signal on its second compare.

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

- A transfer ended by the timer's deadline on the board; QEMU covers it.
- Pins and clocks set by a component of the system rather than the boot
  loader.
- More than one client of the controller, or a third of the timer.
- Both of the timer's clients waiting at once on the board: the BMP280
  domain sleeps only between transfers, so here they never overlap. The
  timer driver's host tests cover two compares armed together.
- A release build of the kernel and domains.
