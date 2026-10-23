#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# Run WORK/out/sensor.img (from build.sh image WORK) on QEMU's imx8mp-evk,
# with I2C4 empty and with a BMP280's registers on it, and check the
# consumer's summary each time.
#
#   qemu.sh WORK
#
# Where QEMU's imx8mp-evk differs from the RSB-3720 in ways that matter:
# - Its GIC has no security extensions and nothing plays the secure
#   firmware, so every interrupt resets to group 0, which seL4 leaves alone
#   on this platform. qemu/gic-shim.S puts them in group 1 and enters the
#   loader.
# - Its I2C controller completes a byte as soon as it is written and sets
#   no flag when an address is refused. An empty bus therefore ends each
#   transfer in the timer domain's timeout, not a NAK, and with a device the
#   controller domain never has to block for an interrupt.
# - Its GPT runs the oscillator source at 16 MHz and ignores the oscillator
#   prescaler; the timer domain measures the rate in any case.
# The BMP280 is QEMU's at24c EEPROM holding a module's registers, the same
# as in sensor-imx-i2c's tests: it answers the driver's register reads and
# ignores conversions.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=../../kernel_architecture/sel4/pins.sh
source "$script_dir/../../kernel_architecture/sel4/pins.sh"

[[ $# -eq 1 ]] || { echo "usage: $0 WORK" >&2; exit 2; }
work=$(cd -- "$1" && pwd)
docker_work=${DOCKER_WORKSPACE:-$work}
[[ -f $work/out/sensor.img ]] || { echo "No $work/out/sensor.img: run build.sh image first" >&2; exit 1; }
mkdir -p -- "$work/out/qemu"
cp -- "$script_dir/qemu/gic-shim.S" "$script_dir/qemu/memory.dts" "$work/out/qemu/"

docker run --rm --user "$(id -u):$(id -g)" --mount "type=bind,src=$docker_work,dst=/work" \
    -w /work/out/qemu "$image" sh -ec '
    aarch64-linux-gnu-as -o gic-shim.o gic-shim.S
    aarch64-linux-gnu-ld -Ttext=0x40800000 -o gic-shim.elf gic-shim.o
    aarch64-linux-gnu-objcopy -O binary gic-shim.elf gic-shim.bin
    mkimage -q -A arm64 -O linux -T kernel -C none -a 0x40800000 -e 0x40800000 \
        -n gic-shim -d gic-shim.bin gic-shim.uimg >/dev/null
    dtc -q -I dts -O dtb -o memory.dtb memory.dts
    python3 - <<PY
regs = bytearray(512)  # one sector; the chip uses the first 256 bytes
regs[0xD0] = 0x58      # chip ID
regs[0x88:0xA0] = bytes.fromhex("906d6e6332004d90c0d5d00b311bc8fff9ff8c3cf8c67017")
regs[0xF7:0xFD] = bytes.fromhex("551600868c00")
open("bmp280-regs.bin", "wb").write(regs)
PY
    run() {
        timeout 60 qemu-system-aarch64 -M imx8mp-evk -m 6G -display none -monitor none \
            -serial null -serial null -serial stdio \
            -kernel gic-shim.uimg -dtb memory.dtb \
            -device loader,file=/work/out/sensor.img,addr=0x41000000,force-raw=on "$@" || true
    }
    run > empty.log 2>&1
    run -drive if=none,id=bmp,file=bmp280-regs.bin,format=raw \
        -device at24c-eeprom,bus=i2c-bus.3,address=0x76,rom-size=512,address-size=1,drive=bmp \
        > bmp280.log 2>&1
'

check() { # log pattern what
    if grep -aq -- "$2" "$work/out/qemu/$1"; then
        echo "ok: $3"
    else
        echo "FAILED: $3 (see $work/out/qemu/$1)"
        status=1
    fi
}
status=0
check empty.log 'SENSOR_RESULT: samples=0 nak=0 timeout=5 other=0 of 5' \
    'empty bus: every transfer ends in the timer domain'"'"'s timeout'
check empty.log 'I2C_WAITS: transfers=5 irq=0 timer=[1-9][0-9]* polled=0' \
    'empty bus: the timer, not the controller, ended every wait'
check bmp280.log 'SENSOR_RESULT: samples=5 nak=0 timeout=0 other=0 of 5' \
    'BMP280 registers on the bus: 5 samples'
check bmp280.log 'pressure 100526390e-3 Pa temperature 3106e-2 C' \
    'the values the host tests compute from the same registers'
check bmp280.log 'I2C_WAITS: transfers=20 irq=0 timer=0 polled=0' \
    'BMP280 registers on the bus: no byte was waited for'
exit $status
