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
build=build-zero-copy

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
    rm -rf -- "$workspace/sel4test/projects/zero_copy"
    cp -r -- "$script_dir/zero_copy" "$workspace/sel4test/projects/zero_copy"
    mkdir -p -- "$workspace/sel4test/$build"
    "${container[@]}" "$image" sh -c \
        'cmake -G Ninja -DCROSS_COMPILER_PREFIX=riscv64-unknown-elf- \
            -C ../projects/zero_copy/settings.cmake ../projects/zero_copy &&
         ninja -j"$BUILD_JOBS"' \
        2>&1 | tee "$workspace/logs/$build.log"
else
    [[ -d $workspace/sel4test/$build ]] || {
        echo "No build at $workspace/sel4test/$build: run '$0 build $2' first" >&2; exit 1;
    }
    "${container[@]}" -i "$image" python3 - \
        "/work/logs/$build-test.log" "${TEST_TIMEOUT:-120}" \
        < "$script_dir/zero_copy/capture.py"
fi
