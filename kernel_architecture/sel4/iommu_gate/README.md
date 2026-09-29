# DMA isolation gate on QEMU RISC-V

Status: 16/16 checks pass on QEMU 10.0.11; emulated IOMMU only, no hardware · Updated: 2026-09-21

A seL4 root task that programs QEMU's RISC-V IOMMU (`riscv-iommu-pci`) and
drives two QEMU `edu` devices as untrusted bus masters. It checks whether a
device can read, write or deliver an interrupt outside what it was granted,
and whether a revoked grant is really gone.

## Run

Uses the workspace and pins of `sel4test.sh`; prepare it once:

```sh
bash sel4test.sh prepare /tmp/sel4-work
bash iommu_gate.sh build /tmp/sel4-work
bash iommu_gate.sh test /tmp/sel4-work
```

`build` copies this directory into the workspace, because the container sees
only the workspace. The test saves the complete serial log to
`logs/build-iommu-gate-test.log`. It passes only when the log contains the
success marker, a non-empty summary with every check passed, and no `FAIL:`
line. Early exit, failure and timeout return nonzero. Overrides:
`DOCKER_WORKSPACE` and `BUILD_JOBS` as for `sel4test.sh`, and `TEST_TIMEOUT`,
default 120 seconds.

QEMU configuration (`capture.py`): `virt` without AIA, `rv64`, 1 hart,
3072MiB, `riscv-iommu-pci` at 00:01.0, and `edu,dma_mask=0xffffffffff` at
00:02.0 and 00:03.0. The wide DMA mask lets a device emit full physical
addresses, as a hostile driver would.

## Checks

Refusal is judged two ways: the target memory is unchanged, and the IOMMU
fault queue holds records with the expected cause and requester ID, and the
expected address where the cause carries one (context and MSI-table faults
report none). QEMU writes several records per refused transfer (16 per 64
bytes); every record must match. Permitted transfers must leave the fault
queue empty.

| Check | Expected result |
|---|---|
| Write while the device directory is Off | refused, cause 256 |
| Device with no context, aimed at the other device's granted IOVA and at its physical page | refused, cause 258; page unchanged |
| Read through a read-write mapping | permitted, no fault |
| Write back through the same mapping | data arrives, no fault |
| Write to another page's physical address | refused, cause 15; page unchanged |
| Write to an unmapped IOVA | refused, cause 15 |
| Read of another page, then write-out to a mapped page | refused, cause 13; nothing copied |
| Write through a read-only mapping | refused, cause 15; page unchanged |
| Read through a read-only mapping | permitted |
| Second device, given its own domain with its own page at the first device's grant IOVA | each device reaches its own page |
| Second device writing to an IOVA mapped only for the first | refused, cause 15; page unchanged |
| Write after unmap, `IOTINVAL.VMA` and `IOFENCE.C` | refused, cause 15 |
| Read after the same revocation | refused, cause 13; nothing copied |
| MSI to the interrupt file the device was granted | delivered |
| MSI to an interrupt file with no MSI page-table entry | refused, cause 262 |
| MSI addressed to the delivery target's physical address | refused, cause 15 |

The first check runs before the directory exists. 00:02.0 then gets a
domain (read-write, read-only and scratch pages); 00:03.0 has no context
until the two domain-separation checks, when it gets a domain of its own.

The log also records one observation that is not a check: after the leaf is
cleared **without** IOTLB invalidation, a device write still lands, because
QEMU keeps the cached translation. The revocation checks therefore exercise
the invalidation itself, not merely the page-table edit.

## Implementation

- `riscv_iommu.c`: one-level device directory with extended device contexts,
  first-stage Sv39 translation, second stage Bare, one PSCID per domain.
  Command and fault queues are polled. Every context change is followed by
  `IODIR.INVAL_DDT`, every page-table change by `IOTINVAL.VMA`, and each by
  an `IOFENCE.C` with PR and PW set that writes a completion word. Leaf PTEs
  preset A and D because hardware A/D update is not enabled. MSI translation
  uses a flat MSI page table in basic mode.
- `pci.c`: ECAM access for bus 0 function 0, BAR0 placement in the 32-bit
  window, bus mastering, and 64-bit MSI capability programming.
- `edu.c`: identification, DMA engine and interrupt trigger.

Whatever holds the IOMMU registers and tables decides what every device
behind it can reach, so that code belongs to the trusted computing base.

## Limits

- This is QEMU's device model, not hardware. QEMU models no caches, so cache
  maintenance before a buffer is returned is untested, and IOTLB behaviour is
  QEMU's. QEMU executes commands synchronously, so it cannot show whether the
  `IOFENCE.C` wait or its PR/PW ordering is needed; only the `IOTINVAL.VMA`
  is shown to matter. Requests still in flight before the IOMMU would need
  an interconnect-level flush, which is not done.
- One protection domain: the root task owns the IOMMU and also drives the
  devices. A separate, untrusted driver component is not exercised.
- Grants have no deadline or abort path.
- No PCIe switch: peer-to-peer routing and ACS are not modelled.
- MSI delivery targets are RAM pages standing in for interrupt files. AIA is
  off because the seL4 platform uses the PLIC, so no IMSIC receives them.
  MRIF mode is untested.
- Requester IDs are limited to bus 0 and, with a one-level directory, to
  values below 64.

## Sources

- [RISC-V IOMMU Architecture Specification](https://github.com/riscv-non-isa/riscv-iommu),
  version 1.0: registers, device context, command and fault queues, MSI
  page tables.
- QEMU 10.0.11 [`hw/riscv/riscv-iommu.c`](https://github.com/qemu/qemu/blob/v10.0.11/hw/riscv/riscv-iommu.c),
  [`hw/riscv/riscv-iommu-bits.h`](https://github.com/qemu/qemu/blob/v10.0.11/hw/riscv/riscv-iommu-bits.h),
  [`hw/misc/edu.c`](https://github.com/qemu/qemu/blob/v10.0.11/hw/misc/edu.c)
  and [`hw/riscv/virt.c`](https://github.com/qemu/qemu/blob/v10.0.11/hw/riscv/virt.c):
  reset state, context caching, fault reporting, `edu` DMA and MSI
  behaviour, and the memory map.
