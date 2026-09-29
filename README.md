# eco — edge compute open toolkit

Generic, reusable pieces of the SoC / edge / ML-sensor platform: kernel-operation
demos today; sensor drivers, memory, filesystem and inter-driver protocol as they
stabilise. Program-specific strategy and integration stay in the closed `doc`
repo (ADR-0001).

## Status — read this before using anything here

Licensed under BSD-3-Clause ([LICENSE](LICENSE)), with the exceptions below.
The original demos remain stubs. A working [seL4test build and QEMU runner](kernel_architecture/sel4/sel4test_build_instructions.md)
is available for development; it builds and tests pinned upstream code.
The [IOMMU gate](kernel_architecture/sel4/iommu_gate/README.md) tests DMA and
MSI confinement through QEMU's RISC-V IOMMU from a seL4 root task.

### Licensing

The BSD-3-Clause licence covers the contributions of its copyright holder. Files
that a second contributor created, listed below, are **not** covered until that
contributor agrees; until then they remain all rights reserved.

- `kernel_architecture/README.md`
- `kernel_architecture/fuchsia/fidl_hello/`
- `kernel_architecture/linux/`
- `kernel_architecture/sel4/ipc_demo/`
- `kernel_architecture/sel4/sel4test_build_instructions.md` (since rewritten;
  its first version is theirs)

Files derived from upstream projects keep their upstream licence and say so in
an SPDX header. `kernel_architecture/sel4/iommu_gate/CMakeLists.txt` and
`settings.cmake` are BSD-2-Clause, from seL4test.

No publication path is configured. A public copy is made only after the
contributor above agrees and the full history passes a privacy review.

### What the demos actually are

Reviewed 2026-09-10; all content predates 2025-05 apart from CI changes.

| Path | Claim | Reality |
|---|---|---|
| `kernel_architecture/fuchsia/fidl_hello` | FIDL hello-world | Two 4-line `println!` programs. No `BUILD.gn`, no `.cml`, no package. The `.fidl` uses pre-RFC-0050 syntax and will not compile against a current `fidlc`. |
| `kernel_architecture/sel4/ipc_demo` | seL4 IPC demo | **Contains no IPC.** The `.camkes` file declares one component with no procedure and no connection, and is never referenced by the build; `CMakeLists.txt` builds a host-side executable. |
| `kernel_architecture/linux` | Linux module + inspection | The module contains no syscall despite its name; the Makefile breaks under `sudo` (`$(PWD)` → use `$(CURDIR)`); `inspect_modules.sh` prints thousands of lines before filtering and has unanchored greps. The README's examples are x86 desktop (`vmx`, `avx512`, `sha_ni`) and several statements about DDS, kTLS and Bluetooth are wrong. |

Treat the 3 original demos above as **stubs** until repaired or removed.
The seL4test runner is separate from `sel4/ipc_demo` and does not repair it.

## Layout

```
kernel_architecture/
  fuchsia/     FIDL and component demos
  sel4/        seL4 / CAmkES demos
  linux/       module, syscall and inspection demos
```

## Contributing

Not yet: the second contributor's agreement is still open (see Licensing).
