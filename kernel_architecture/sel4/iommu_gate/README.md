# DMA isolation gate on QEMU RISC-V

Status: 25/25 checks pass on QEMU 10.0.11; emulated IOMMU only, no hardware · Updated: 2026-10-03

A seL4 root task that programs QEMU's RISC-V IOMMU (`riscv-iommu-pci`) and
drives two QEMU `edu` devices as untrusted bus masters. It checks whether a
device can read, write or deliver an interrupt outside what it was granted,
whether it can reach the IOMMU's own structures, whether a revoked grant is
really gone, and whether a grant's deadline takes the page back from a device
that does not stop.

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
addresses, as a hostile driver would. The deadline checks read the time from
the machine's own goldfish RTC at `0x101000`.

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
| Write to the device's own context in the device directory | refused, cause 15; context unchanged |
| Write to the IOMMU command queue | refused, cause 15; queue unchanged |
| Write to the device's own page-table root | refused, cause 15; table unchanged |
| Read of the device's own context, then write-out to a mapped page | refused, cause 13; nothing copied |
| Grant with a 1 s deadline, device write | lands before the deadline, no fault |
| Grant with a 20 ms deadline, device write that edu starts 100 ms later | still pending at the deadline |
| Abort: unmap, `IOTINVAL.VMA`, `IOFENCE.C` | done while the transfer is still pending |
| The device's late write, after the page holds its next owner's data | refused, cause 15; the new data intact |
| The same for a late read, then write-out to a mapped page | refused, cause 13; nothing of the new data copied |

The first check runs before the directory exists. 00:02.0 then gets a
domain (read-write, read-only and scratch pages); 00:03.0 has no context
until the two domain-separation checks, when it gets a domain of its own.

Checks 17 to 20 aim the device at the structures that confine it. Check 5
already covers the class (no mapping, no access), but these are the targets
that matter on silicon: a device that could rewrite its own context or page
table could grant itself anything. The device's buffer holds a known pattern
first, so a write that landed would show.

## Deadlines

Every grant carries a deadline, and expiry takes the page back without the
device's cooperation. QEMU's `edu` performs a transfer in one piece about
100 ms (virtual clock) after its command, whatever happens meanwhile, so a
transfer started under a grant with a 20 ms deadline is still pending when the
deadline passes: a device that ignores the revoke. The manager then runs the
return sequence (unmap, `IOTINVAL.VMA`, `IOFENCE.C`) and gives the page to its
next owner, and the device's transfer runs afterwards and must be refused.
Before the late transfer starts, the device has used the grant once, so the
IOMMU holds a cached translation: without the `IOTINVAL.VMA` in the abort, the
late write lands. QEMU models no caches, so the sequence has no cache
maintenance step.

Mutants checked on 2026-10-03, each failing the run at the check aimed at it:
the abort without IOTLB invalidation; no abort before a late write; no abort
before a late read; a deadline longer than the device takes; and the device's
own context page mapped into its domain.

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
- `edu.c`: identification, DMA engine (a transfer can be started and left
  running) and interrupt trigger.

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
- A deadline abort is shown against a transfer that has not started. `edu`
  performs each transfer in one piece, so a transfer already in progress
  across the revoke, half before and half after, cannot be shown.
- The deadline is polled against the RTC in the single task. A manager with
  other work would take an alarm interrupt instead, as the zero-copy handoff
  does.
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
