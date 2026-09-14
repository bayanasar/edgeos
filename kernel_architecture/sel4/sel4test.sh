#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "Usage: $0 {prepare|build|test} WORKSPACE [qemu-riscv64|imx8mp]" >&2
    echo "Environment: REPO_TOOL, DOCKER_WORKSPACE, DTS_OVERLAY, TEST_TIMEOUT, BUILD_JOBS" >&2
    exit 2
}

[[ $# -ge 2 && $# -le 3 ]] || usage
action=$1
case "$action" in prepare|build|test) ;; *) usage ;; esac
workspace=$(realpath -m -- "$2")
profile=${3:-qemu-riscv64}
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
manifest=ed9334e4f7f2b4eb2fe8fba0338e677ce35d8ec2
image=trustworthysystems/sel4@sha256:ec5e639ed64c3033e86d90188aebf16a41933fd801f772a3da35ff77c6885c5d

case "$profile" in
    qemu-riscv64)
        build=build-qemu-riscv64-elf
        config=(-DPLATFORM=qemu-riscv-virt -DRISCV64=1 -DSIMULATION=TRUE
                -DCROSS_COMPILER_PREFIX=riscv64-unknown-elf-)
        ;;
    imx8mp)
        build=build-imx8mp-baseline
        config=(-DPLATFORM=imx8mp-evk -DAARCH64=1
                -DCROSS_COMPILER_PREFIX=aarch64-linux-gnu-)
        ;;
    *) usage ;;
esac
config+=(-DRELEASE=OFF -DVERIFICATION=OFF -DSMP=OFF -DMCS=OFF
         -DDOMAINS=OFF -DBAMBOO=OFF '-DLibSel4TestPrinterRegex=.*')

if [[ -n ${DTS_OVERLAY:-} ]]; then
    [[ $profile == imx8mp ]] || { echo "DTS_OVERLAY is only supported for imx8mp" >&2; exit 2; }
    overlay=$(realpath -e -- "$workspace/$DTS_OVERLAY")
    [[ $overlay == "$workspace/"* ]] || { echo "Overlay must be inside WORKSPACE" >&2; exit 2; }
    config+=("-DKernelCustomDTSOverlay=/work/${overlay#"$workspace/"}")
    build=build-imx8mp-overlay
fi

if [[ $action == prepare ]]; then
    mkdir -p -- "$workspace/sel4test" "$workspace/logs"
    cd -- "$workspace/sel4test"
    "${REPO_TOOL:-repo}" init -u https://github.com/seL4/sel4test-manifest.git \
        -b "$manifest" --depth=1 --no-clone-bundle
    "${REPO_TOOL:-repo}" sync -c -j"${BUILD_JOBS:-4}" --no-clone-bundle
    "${REPO_TOOL:-repo}" manifest -r -o "$workspace/manifest.lock.xml"
    exit 0
fi

actual=$(git -C "$workspace/sel4test/.repo/manifests" rev-parse HEAD)
[[ $actual == "$manifest" ]] || {
    echo "Manifest mismatch: run prepare in a fresh workspace" >&2; exit 1;
}
mkdir -p -- "$workspace/logs"
docker_workspace=${DOCKER_WORKSPACE:-$workspace}
container=(docker run --rm --user "$(id -u):$(id -g)" -e CCACHE_DIR=/work/.ccache
    -e "BUILD_JOBS=${BUILD_JOBS:-4}"
    --mount "type=bind,src=$docker_workspace,dst=/work"
    -w "/work/sel4test/$build")

if [[ $action == build ]]; then
    mkdir -p -- "$workspace/sel4test/$build"
    "${container[@]}" "$image" sh -c \
        '../init-build.sh "$@" && ninja -j"$BUILD_JOBS"' sh "${config[@]}" \
        2>&1 | tee "$workspace/logs/$build.log"
else
    [[ $profile == qemu-riscv64 ]] || {
        echo "The imx8mp profile is build-only; hardware boot needs a board console" >&2; exit 2;
    }
    "${container[@]}" -i "$image" python3 - \
        "/work/logs/$build-test.log" "${TEST_TIMEOUT:-180}" \
        < "$script_dir/qemu_capture.py"
fi
