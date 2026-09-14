# Build and test seL4

Status: QEMU RV64 baseline tested; hardware profiles build-only.

`sel4test.sh` prepares a pinned upstream seL4test tree, builds it in a pinned
seL4 container, and runs the upstream suite in QEMU. It does not implement a
new kernel or repair the separate `ipc_demo` stub.

Requirements: Bash, Git, Docker, and the upstream `repo` launcher. Python and
pexpect run inside the container. Run from this directory:

```sh
bash sel4test.sh prepare /tmp/sel4-work
bash sel4test.sh build /tmp/sel4-work qemu-riscv64
bash sel4test.sh test /tmp/sel4-work qemu-riscv64
```

The first build downloads the container if needed. `prepare` pins manifest
`ed9334e4f7f2b4eb2fe8fba0338e677ce35d8ec2`; the script pins the container by
digest. Source trees, a resolved manifest, build products and logs stay in
the chosen workspace, outside this repository. Use a fresh workspace when
changing the pinned manifest. `REPO_TOOL` can name a repo launcher outside PATH.

The tested QEMU configuration is RV64, 1 hart, 3072MiB RAM, simulation mode,
non-MCS, debug build, with no test-name filter. The initial run passed 116 tests
and disabled 51 according to configuration. Disabled tests are not passes.
`qemu_capture.py` requires both a nonempty passing suite summary and the upstream
success marker; failure, premature exit and timeout return nonzero. It saves
the full output under `logs/` and stops QEMU after the verdict.

Overrides:

- `BUILD_JOBS`: build/sync concurrency, default 4.
- `TEST_TIMEOUT`: positive timeout in seconds, default 180.
- `DOCKER_WORKSPACE`: workspace path as seen by the Docker daemon, when it
  differs from the shell's path. It must refer to the same directory.

## ARM build profile

```sh
bash sel4test.sh build /tmp/sel4-work imx8mp
```

For a board-specific overlay, copy it inside the workspace and specify its
relative path. This uses a separate build directory:

```sh
DTS_OVERLAY=board.dts bash sel4test.sh build /tmp/sel4-work imx8mp
```

This profile compiles an `imx8mp-evk` image; it does not flash or boot hardware.
Board-specific memory reservations, console wiring and bootloader handoff must
be verified separately. QEMU baseline success does not validate this profile,
RVV state isolation, or DMA confinement. No RISC-V IOMMU controller or DMA
isolation test is supplied here.

References: [seL4test](https://docs.sel4.systems/projects/sel4test/),
[official build containers](https://docs.sel4.systems/projects/dockerfiles/),
[i.MX8MP platform](https://docs.sel4.systems/Hardware/imx8mp.html).
