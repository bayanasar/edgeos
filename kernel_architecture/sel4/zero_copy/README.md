# Zero-copy buffer handoff on seL4

Status: step 1 of 4 — enforced exclusivity towards one client, 13/13 checks on QEMU 10.0.11; no timing yet · Updated: 2026-10-01

A buffer manager (the root task) lends a frame to a client in its own address
space and takes it back, so the client's data reaches the next holder without
being copied and the previous holder keeps no access. This is the "enforced"
ownership model: a trusted manager grants a copy of the frame capability per
handoff and revokes on return.

## Run

Uses the workspace and pins of `sel4test.sh`; prepare it once:

```sh
bash sel4test.sh prepare /tmp/sel4-work
bash zero_copy.sh build /tmp/sel4-work
bash zero_copy.sh test /tmp/sel4-work
```

The test saves the serial log to `logs/build-zero-copy-test.log` and passes
only on the success marker, a non-empty summary with every check passed, and no
`FAIL:` line. Overrides as for `iommu_gate.sh`. QEMU: `virt`, `rv64`, 1 hart,
3072MiB, no devices.

## How a handoff works

- The manager allocates the buffer frame and never maps it into itself. The
  contents belong to the clients; the manager decides ownership from its
  capabilities, never from what the buffer holds.
- **Grant:** copy the frame capability with the rights the client is to have
  (read-write, or read-only), and map the copy at a fixed address in the
  client's address space. The command naming the buffer goes to the client as
  the reply to its last call.
- **Take back:** revoke the manager's original capability. In seL4 a copied
  capability is a child of its source, so the revoke deletes the client's
  copy, and deleting a mapped frame capability unmaps it.
- A client access that faults sends the fault to the manager. The manager maps
  a quarantine page at that address and resumes the client, so the access
  completes harmlessly and the test can continue with the same client.

## Checks

Each refusal is paired with the same access while granted, which must work,
so a probe that could never succeed cannot pass as a refusal.

| Check | Expected |
|---|---|
| Client starts in its own address space | it calls the manager |
| Read-write grant, client fills the buffer | checksum of what it reads back matches the pattern |
| Write while granted | succeeds |
| Revoke the original | the client's copy is gone (`Page_GetAddress` fails on its slot) |
| Write after revocation | VM fault at the buffer address |
| The faulted write, resumed | completes on the quarantine page |
| Read after revocation | VM fault at the buffer address |
| Read-only grant of the same frame | contents unchanged since the client wrote them |
| Read under the read-only grant | succeeds |
| Write under the read-only grant | VM fault |
| Revoke the read-only grant | the copy is gone |
| Read the buffer again | no faulted write reached it |

Mutants checked on 2026-10-01: skipping the revoke, mapping the read-only grant
writable, and resuming a faulted write on the buffer instead of the quarantine
page each fail the run.

## Not yet covered

- Two clients, producer and consumer, with a ring of buffers (step 2).
- Cost per handoff against a copy (step 3). QEMU's timing is not hardware
  timing, so absolute costs need a real board.
- Revocation on a deadline when a client does not return the buffer (step 4).
- Device DMA. This is a CPU-only path: no IOMMU invalidation or cache
  maintenance is involved, and a single hart means no cross-core TLB shootdown.
- Architectures other than RISC-V: the mapping calls are `seL4_RISCV_*`.
