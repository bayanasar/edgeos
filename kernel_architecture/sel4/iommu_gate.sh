#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "Usage: $0 {build|test} WORKSPACE" >&2
    echo "WORKSPACE must be prepared by: sel4test.sh prepare WORKSPACE" >&2
    echo "Environment: DOCKER_WORKSPACE, TEST_TIMEOUT, BUILD_JOBS" >&2
    exit 2
}

[[ $# -eq 2 ]] || usage
action=$1
case "$action" in build|test) ;; *) usage ;; esac
workspace=$(realpath -m -- "$2")
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$script_dir/pins.sh"
build=build-iommu-gate

actual=$(git -C "$workspace/sel4test/.repo/manifests" rev-parse HEAD 2>/dev/null || true)
[[ $actual == "$manifest" ]] || {
    echo "Manifest mismatch: run 'sel4test.sh prepare' in a fresh workspace" >&2; exit 1;
}
mkdir -p -- "$workspace/logs"
docker_workspace=${DOCKER_WORKSPACE:-$workspace}
container=(docker run --rm --user "$(id -u):$(id -g)" -e CCACHE_DIR=/work/.ccache
    -e "BUILD_JOBS=${BUILD_JOBS:-4}"
    --mount "type=bind,src=$docker_workspace,dst=/work"
    -w "/work/sel4test/$build")

if [[ $action == build ]]; then
    # The container sees only the workspace, so stage the sources into it.
    rm -rf -- "$workspace/sel4test/projects/iommu_gate"
    cp -r -- "$script_dir/iommu_gate" "$workspace/sel4test/projects/iommu_gate"
    mkdir -p -- "$workspace/sel4test/$build"
    "${container[@]}" "$image" sh -c \
        'cmake -G Ninja -DCROSS_COMPILER_PREFIX=riscv64-unknown-elf- \
            -C ../projects/iommu_gate/settings.cmake ../projects/iommu_gate &&
         ninja -j"$BUILD_JOBS"' \
        2>&1 | tee "$workspace/logs/$build.log"
else
    "${container[@]}" -i "$image" python3 - \
        "/work/logs/$build-test.log" "${TEST_TIMEOUT:-120}" \
        < "$script_dir/iommu_gate/capture.py"
fi
