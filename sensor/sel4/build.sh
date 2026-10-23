#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# Build the sensor protection domains into a Microkit image for the RSB-3720.
#
#   build.sh sdk   WORK   Microkit SDK for the RSB-3720, in WORK/microkit
#   build.sh image WORK   WORK/out/sensor.img from rsb3720.system
#
# Both steps run in the pinned seL4 build image. WORK holds the sources,
# a Rust toolchain of its own and the outputs; nothing is installed on the
# host. Environment: DOCKER_WORKSPACE (WORK as the Docker daemon sees it,
# if different), BUILD_JOBS.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
sensor_dir=$(dirname -- "$script_dir")
# shellcheck source=../../kernel_architecture/sel4/pins.sh
source "$sensor_dir/../kernel_architecture/sel4/pins.sh"

microkit_rev=8780fab8699f5aeec109b21325ff37c741736b24 # Microkit 2.3.0
sel4_rev=6e7c3b733d296cfd88d5fbf635c96e447a882374     # seL4 16.0.0
toolchain=nightly-2026-08-21
board=rsb3720
config=debug
rust_sel4_rev=$(sed -n 's/^sel4 = .*rev = "\([0-9a-f]*\)".*/\1/p' "$script_dir/Cargo.toml")

usage() {
    echo "usage: $0 sdk|image WORK" >&2
    exit 2
}
[[ $# -eq 2 ]] || usage
action=$1
mkdir -p -- "$2"
work=$(cd -- "$2" && pwd)
docker_work=${DOCKER_WORKSPACE:-$work}

# The container sees only WORK, at /work, so sources are staged into it.
container=(docker run --rm --user "$(id -u):$(id -g)"
    --mount "type=bind,src=$docker_work,dst=/work"
    -e HOME=/work/home -e RUSTUP_HOME=/work/toolchains/rustup -e CARGO_HOME=/work/toolchains/cargo
    -e PATH=/work/toolchains/cargo/bin:/usr/local/bin:/usr/bin:/bin
    -e RUSTUP_TOOLCHAIN=$toolchain -e "BUILD_JOBS=${BUILD_JOBS:-4}")

fetch() { # url rev dir
    if [[ ! -d $work/$3/.git ]]; then
        git init -q "$work/$3"
        git -C "$work/$3" remote add origin "$1"
    fi
    git -C "$work/$3" fetch -q --depth 1 origin "$2"
    git -C "$work/$3" checkout -q --force --detach FETCH_HEAD
    git -C "$work/$3" clean -qfdx
}

case $action in
sdk)
    fetch https://github.com/seL4/microkit.git "$microkit_rev" microkit
    fetch https://github.com/seL4/seL4.git "$sel4_rev" seL4
    git -C "$work/microkit" apply "$script_dir/microkit-rsb3720.patch"
    cp -- "$sensor_dir/../kernel_architecture/sel4/boards/rsb3720.dts" \
        "$work/microkit/custom_dts/rsb3720.dts"
    mkdir -p -- "$work/home"
    "${container[@]}" -w /work/microkit "$image" sh -ec "
        if ! command -v rustup >/dev/null; then
            curl -sSf https://sh.rustup.rs -o /work/home/rustup-init.sh
            sh /work/home/rustup-init.sh -y --no-modify-path --profile minimal \
                --default-toolchain $toolchain
        fi
        rustup toolchain install $toolchain --profile minimal --component rust-src \
            --target aarch64-unknown-none --target x86_64-unknown-linux-musl
        python3 build_sdk.py --sel4 /work/seL4 --boards $board --configs $config \
            --skip-docs --skip-tar --gcc-toolchain-prefix-aarch64 aarch64-linux-gnu
    "
    ;;
image)
    sdk=/work/microkit/release/microkit-sdk-2.3.0
    [[ -x $work/microkit/release/microkit-sdk-2.3.0/bin/microkit ]] || {
        echo "No SDK in $work: run '$0 sdk $work' first" >&2
        exit 1
    }
    rm -rf -- "$work/src" && mkdir -p -- "$work/src" "$work/out"
    tar -C "$(dirname -- "$sensor_dir")" --exclude=target -cf - sensor | tar -C "$work/src" -xf -
    "${container[@]}" -w /work/src/sensor/sel4 "$image" sh -ec "
        SEL4_INCLUDE_DIRS=$sdk/board/$board/$config/include \
            cargo build --release --locked --target-dir /work/target --artifact-dir /work/out
        # i2c-imx waits on its input endpoint by slot number, which neither
        # the SDK nor sel4-microkit exports: check both still use slot 1.
        grep -qF 'PD_INPUT_CAP_IDX: u64 = 1;' /work/microkit/tool/microkit/src/capdl/builder.rs &&
            grep -qF 'INPUT_CAP: sel4::cap::Endpoint = sel4::Cap::from_bits(1);' \
                /work/toolchains/cargo/git/checkouts/rust-sel4-*/${rust_sel4_rev:0:7}/crates/sel4-microkit/base/src/ipc.rs || {
            echo 'Microkit input endpoint not in slot 1: change INPUT in i2c-imx' >&2
            exit 1
        }
        $sdk/bin/microkit rsb3720.system --search-path /work/out --board $board \
            --config $config -r /work/out/report.txt -o /work/out/sensor.img
    "
    echo "Image: $work/out/sensor.img"
    ;;
*) usage ;;
esac
