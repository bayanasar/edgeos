# Zero-copy buffer handoff on seL4

Status: steps 1 and 2 of 4 — one client, then producer to consumer through a ring; 22/22 checks on QEMU 10.0.11; no timing yet · Updated: 2026-10-01

A buffer manager (the root task) lends frames to clients in their own address
spaces and takes them back, so a producer's data reaches a consumer without
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
- With two clients, the manager keeps each one's reply capability
  (`seL4_CNode_SaveCaller`, non-MCS) so it can answer them in any order.
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
| Consumer starts in its own address space | it calls the manager |
| 32 handoffs through a ring of 4 buffers: producer fills (read-write), revoke, consumer reads (read-only), revoke | every consumer checksum equals the producer's and the expected pattern |
| Producer fills buffer 0, writes while holding it | succeeds |
| Producer writes, then reads, while the consumer holds buffer 0 | both fault |
| Consumer reads buffer 0 | the producer's data, untouched by its refused write |
| Consumer reads buffer 1, which it does not hold | faults |
| Consumer writes the buffer it holds | faults |
| Consumer reads buffer 0 again | intact after every refusal |

Mutants checked on 2026-10-01, each failing the run at the check aimed at
it: skipping the revoke; mapping the read-only grant writable; resuming a
faulted write on the buffer instead of the quarantine page; leaving the
producer's mapping in place while the consumer holds the buffer; granting the
consumer write access; giving the consumer a second buffer; and the consumer
reading the wrong slot of the ring.

## Not yet covered

- Handoffs are sequential: the producer does not fill the next buffer while
  the consumer reads one.
- Cost per handoff against a copy (step 3). QEMU's timing is not hardware
  timing, so absolute costs need a real board.
- Revocation on a deadline when a client does not return the buffer (step 4).
- Device DMA. This is a CPU-only path: no IOMMU invalidation or cache
  maintenance is involved, and a single hart means no cross-core TLB shootdown.
- Architectures other than RISC-V: the mapping calls are `seL4_RISCV_*`.
