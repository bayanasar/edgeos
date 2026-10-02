# Kernel architecture

seL4 tooling, in `sel4/`:

- a pinned seL4test build and QEMU runner ([instructions](sel4/sel4test_build_instructions.md));
- the [IOMMU gate](sel4/iommu_gate/README.md), a root task that tests DMA and
  MSI confinement through QEMU's RISC-V IOMMU;
- device-tree overlays for boards in `sel4/boards/`.
