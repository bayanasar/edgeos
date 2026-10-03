# eco — edge compute open toolkit

Generic, reusable pieces of an SoC / edge / ML-sensor platform: seL4 tooling
today; sensor drivers, memory, filesystem and inter-driver protocol as they
stabilise.

## Status — read this before using anything here

Licensed under BSD-3-Clause ([LICENSE](LICENSE)). A working [seL4test build and QEMU runner](kernel_architecture/sel4/sel4test_build_instructions.md)
is available for development; it builds and tests pinned upstream code.
The [IOMMU gate](kernel_architecture/sel4/iommu_gate/README.md) tests DMA and
MSI confinement through QEMU's RISC-V IOMMU from a seL4 root task. The
[zero-copy handoff](kernel_architecture/sel4/zero_copy/README.md) passes
buffers between isolated clients on seL4 without copying them, and takes them
back by revoking the capability.

### Licensing

The BSD-3-Clause licence covers every file in the current tree. Earlier
versions of some paths were written by a second contributor: the original
demos, which were stubs and have been removed, and the first versions of
`kernel_architecture/README.md` and
`kernel_architecture/sel4/sel4test_build_instructions.md`, both since rewritten
with none of their text. Those versions remain in the history, all rights
reserved, and are not covered by the licence.

Files derived from upstream projects keep their upstream licence and say so in
an SPDX header. `kernel_architecture/sel4/iommu_gate/CMakeLists.txt` and
`settings.cmake` are BSD-2-Clause, from seL4test.

This repository is public, every branch included: a branch is published as
soon as it is pushed, so its name, commit messages and authorship are public
from the first push.

## Layout

```
kernel_architecture/
  sel4/        seL4test build and QEMU runner, IOMMU gate, zero-copy handoff,
               RSB-3720 overlay
sensor/        portable sensor drivers in Rust; sensor-core interface
```

## Contributing

Not yet open to outside contributions.
