# eco — edge compute open toolkit

Generic, reusable pieces of the SoC / edge / ML-sensor platform: kernel-operation
demos today; sensor drivers, memory, filesystem and inter-driver protocol as they
stabilise. Program-specific strategy and integration stay in the closed `doc`
repo (ADR-0001).

## Status — read this before using anything here

**This repository has no repository-wide licence.** The original demos below
remain stubs. A working [seL4test build and QEMU runner](kernel_architecture/sel4/sel4test_build_instructions.md)
is now available for development; it builds and tests pinned upstream code.

### Licensing (blocking)

There is **no LICENSE file**. Under default copyright that means *all rights
reserved* — nobody may use, copy or redistribute this, whatever the word "open"
in the name suggests. The repository also has more than one contributor, so a
licence cannot simply be declared after the fact: it needs the other
contributor's agreement, or a DCO going forward.

**Licence choice is the owner's decision and has not been made.** Public
distribution remains blocked on licensing; development tooling is maintained
here while that decision is pending.

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

Not yet — see Licensing above.
