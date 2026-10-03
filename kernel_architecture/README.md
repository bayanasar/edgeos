# Kernel architecture

seL4 tooling, in `sel4/`:

- a pinned seL4test build and QEMU runner ([instructions](sel4/sel4test_build_instructions.md));
- the [IOMMU gate](sel4/iommu_gate/README.md), a root task that tests DMA and
  MSI confinement through QEMU's RISC-V IOMMU;
- a [zero-copy buffer handoff](sel4/zero_copy/README.md): a buffer manager
  lends frames to clients and revokes them, with the cost per handoff
  counted by the kernel and revocation on a deadline;
- device-tree overlays for boards in `sel4/boards/`.
