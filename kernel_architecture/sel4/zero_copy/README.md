# Zero-copy buffer handoff on seL4

Status: all 4 steps — one client, producer to consumer through a ring, the cost per handoff counted by the kernel and timed on an RSB-3720, and revocation on a deadline; 37/37 checks on QEMU 10.0.11, 30/30 on the board · Updated: 2026-10-21

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

For the RSB-3720 (i.MX 8M Plus), `bash zero_copy.sh build /tmp/sel4-work rsb3720`
builds `images/zero_copy-image-arm-imx8mp-evk` in `build-zero-copy-rsb3720`:
the `imx8mp-evk` platform with `boards/rsb3720.dts`, a release kernel that
still prints, and user access to the generic timer's virtual counter. There
step 3 times the handoffs instead of counting kernel entries, and step 4 is
not built (its timer is QEMU's). The image is a raw binary whose first byte
belongs at the elfloader's entry point; load it there from U-Boot and `go`
to the same address (`readelf -h build-zero-copy-rsb3720/elfloader/elfloader`).
The board runs 30 checks: step 3 replaces its four count checks with three
timing checks, and step 4 is not built.

The test saves the serial log to `logs/build-zero-copy-test.log` and passes
only on the success marker, a non-empty summary with every check passed, and no
`FAIL:` line. Overrides as for `iommu_gate.sh`. QEMU: `virt`, `rv64`, 1 hart,
3072MiB, no added devices; step 4 uses the machine's own goldfish RTC. The
kernel is built with `KernelBenchmarks` set to `track_kernel_entries` for
step 3 (`settings.cmake`). The log shows a kernel line "Attempted to invoke a
null cap" each time a take-back checks that the client's slot is empty; those
are expected.

## How a handoff works

- The manager allocates the buffer frame and never maps it into itself. The
  contents belong to the clients; the manager decides ownership from its
  capabilities, never from what the buffer holds.
- **Grant:** copy the frame capability with the rights the client is to have
  (read-write, or read-only), and map the copy at a fixed address in the
  client's address space. The command naming the buffer goes to the client as
  the reply to its last call.
- **Take back:** revoke the manager's original capability. The original is
  revocable because it came from `Untyped_Retype`, and the copy covers the
  same region, so `Revoke` on the original deletes the copy; deleting a mapped
  frame capability unmaps it. The copy itself is not revocable: on RISC-V,
  `Arch_isCapRevocable` returns false for every frame capability, whether
  copied or minted. Revoking a copy therefore deletes nothing, and only the
  holder of the original can take a buffer back. A manager that delegates to
  a second-level manager cannot let the second level revoke its own grants;
  the first level can only revoke the whole region.
- **Page tables** that a grant creates in a client are never freed. Here the
  addresses are reused, so the number is bounded; a long-running manager
  would have to reclaim them.
- With two clients, the manager keeps each one's reply capability
  (`seL4_CNode_SaveCaller`, non-MCS) so it can answer them in any order.
- A client access that faults sends the fault to the manager. The manager maps
  a quarantine page at that address and resumes the client, so the access
  completes harmlessly and the test can continue with the same client. This
  is harness policy, not production policy: a real manager would stop or
  restart a client that faults, not give it a writable page to finish the
  access.

## Checks

Each refusal is paired with the same access while granted, which must work,
so a probe that could never succeed cannot pass as a refusal.

| Check | Expected |
|---|---|
| Client starts in its own address space | it calls the manager |
| Read-write grant, client fills the buffer | checksum of what it reads back matches the pattern |
| Write while granted | succeeds |
| Revoke the client's copy, which holds a copy of its own | nothing is deleted: both copies remain, and the client still reads the buffer |
| Revoke the original | the client's copy is gone (`Page_GetAddress` fails on its slot) |
| The copy of the copy | gone too |
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
| Step 3, each scheme at 4 KiB, 64 KiB and 2 MiB | every handoff arrives intact |
| Kernel log of each window | nothing but the reset, the expected calls and interrupts |
| IPC per command, every scheme | 3 IPC entries and 1 `SaveCaller` |
| Capability operations | `Copy`, `Map` and `Revoke` once per frame on each side, enforced scheme only |
| Bytes copied | the buffer's length per handoff, copier only |
| Step 4: consumer answers within its deadline | normal answer, no abort |
| Consumer holds the buffer (keeps reading, never answers) | the wait ends at the deadline, not before |
| Abort | the revoke empties the consumer's slot without its answer |
| The consumer, still reading | faults at the buffer address |
| Resumed on quarantine | the consumer answers again |
| Next handoff after the abort | arrives intact |

Mutants checked on 2026-10-01, each failing the run at the check aimed at
it: skipping the revoke; mapping the read-only grant writable; resuming a
faulted write on the buffer instead of the quarantine page; leaving the
producer's mapping in place while the consumer holds the buffer; granting the
consumer write access; giving the consumer a second buffer; and the consumer
reading the wrong slot of the ring. Step 3 mutants, checked on 2026-10-02: the
copier skipping its copy; the copier copying half the buffer; an extra kernel
call per revoke; the shared scheme without the consumer's mapping; no log
reset before a window; and the enforced scheme not revoking the producer
before granting the consumer. Step 4 mutants, checked on 2026-10-02: no
revoke on expiry; a deadline of zero for the consumer that answers in time;
and the alarm set before the deadline. Waiting for the holding consumer with
no deadline at all hangs the manager, and the run fails on its timeout. Revocation mutants, checked on
2026-10-03: revoking the original where the client's copy should be revoked,
and taking the copy of the copy from another frame.

## Cost per handoff (step 3)

Three schemes move the same buffer from producer to consumer, and the
kernel's own log of its entries (`seL4_BenchmarkResetLog` to
`seL4_BenchmarkFinalizeLog`) counts what each handoff costs. Eight handoffs
per window, after one outside it that creates any page tables the clients
still lack.

- **Enforced:** the handoff above. A grant copies and maps each frame, and a
  take-back revokes it, on each side.
- **Copier:** each client keeps a private buffer for good, and the manager
  copies the producer's into the consumer's. The parties stay isolated, but
  the manager maps both buffers.
- **Shared:** one buffer mapped for good, read-write in the producer and
  read-only in the consumer. Nothing stops the producer writing while the
  consumer reads, so the consumer must trust the producer to stop: a floor
  for the cost, not an isolation scheme.

Per handoff, the same at every run:

| Scheme | Buffer | Kernel entries | of which IPC and `SaveCaller` | `Copy` / `Map` / `Revoke` | Bytes the manager copies |
|---|---|---|---|---|---|
| enforced | 4 KiB, one frame | 14 | 8 | 2 / 2 / 2 | 0 |
| enforced | 64 KiB, 16 frames of 4 KiB | 104 | 8 | 32 / 32 / 32 | 0 |
| enforced | 2 MiB, one frame of 2 MiB | 14 | 8 | 2 / 2 / 2 | 0 |
| copier | 4 KiB, 64 KiB, 2 MiB | 8 | 8 | 0 | 4096, 65536, 2097152 |
| shared | any | 8 | 8 | 0 | 0 |

Every scheme pays the same two commands: per command, the manager's reply
(`seL4_Send` on the saved reply capability), its `seL4_Recv` and
`SaveCaller`, and the client's `seL4_Call`. On top of that, the enforced
scheme pays six capability invocations per frame and copies nothing, and the
copier pays no invocation and copies the whole buffer. The enforced cost
follows the number of frames, not the bytes, so a 2 MiB buffer in one large
frame costs what a 4 KiB one does.

What the counts do not show: the work inside each invocation, above all the
TLB flush when a revoke unmaps a frame, and a copy's cost in time; the board
timing below measures those. The
interrupts in a window are logged too, but they track how long the clients
ran under QEMU, vary between runs, and are left out of the counts.

Two things the kernel log taught:

- On RISC-V the kernel labels only its system-call entries. Interrupt and
  exception entries set no path and are logged as `Entry_Unknown`; ARM and x86
  label them. A client fault in a window would still fail the run, because it
  reaches the manager as a fault answer instead of `OP_DONE`.
- In a debug build, `vka_cspace_free` checks the slot with
  `seL4_DebugCapIdentify`, a kernel entry per free. The step 3 manager reuses
  the slot each revoke empties instead of freeing and allocating one per
  grant, as a long-running manager would.

## Time per handoff on an RSB-3720 (step 3)

The board build times the three schemes instead of counting kernel entries.
It reads the generic timer's virtual counter (`cntvct_el0`, 8 MHz on this
board, so one tick is 125 ns) around each window of 64 handoffs, and around
the grants, revokes and copies inside it. Release kernel, one core, non-MCS;
the clients write and checksum the whole buffer on every handoff.

Nanoseconds per handoff, lowest and highest of three runs:

| Buffer | Enforced: grants and revokes | Copier: the copy | One revoke, per frame | Cheaper |
|---|---|---|---|---|
| 4 KiB, one frame | 7,339–7,406 | 687–718 | 1,941–1,952 | copier, about 10× |
| 64 KiB, 16 frames of 4 KiB | 72,201–72,298 | 12,537–12,732 | 1,057–1,058 | copier, about 6× |
| 2 MiB, one frame of 2 MiB | 7,849–7,884 | 1,015,109–1,033,203 | 1,706–1,873 | enforced, about 130× |

Each scheme's whole handoff is longer than the shared scheme's by about the
figure in the table (at 4 KiB, 63.5 µs enforced and 56.4 µs copier against
55.7 µs shared), so the timed parts are the parts that differ.

- The enforced cost follows frames, as the counts said: one frame costs 7.3
  to 7.9 µs of grants and revokes whether it holds 4 KiB or 2 MiB, and sixteen
  4 KiB frames cost 72 µs, 4.5 µs each.
- A revoke, TLB flush included, takes 1.7 to 2.0 µs for a frame on its own,
  and 1.1 µs per frame when sixteen are revoked in a row.
- The copy follows bytes: 0.17 to 0.19 ns per byte up to 64 KiB, and 0.48 to
  0.49 at 2 MiB.

So copying 4 KiB costs a tenth of lending it, and in 4 KiB frames the enforced
scheme never pays. In one 2 MiB frame it pays once a copy of the payload would
cost more than its 7.9 µs: at the 64 KiB copy rate, above about 40 KiB per
handoff. That point is interpolated between the 4 KiB and 64 KiB sizes, not
measured. Both schemes pay once per handoff, so the rate of handoffs scales
both costs and does not move the point.

## Deadlines (step 4)

Every grant carries a deadline, and expiry takes the buffer back without the
client's cooperation. The timer is the `virt` machine's goldfish RTC at
`0x101000`: a nanosecond counter whose alarm raises interrupt 11. The
manager binds the interrupt's notification to its own thread, so its wait for
a client's call or fault also ends when the alarm fires.

- A consumer that answers within 100 ms finishes as usual, and the alarm is
  cleared.
- A consumer told to hold the buffer keeps reading it and never answers. At the
  deadline the manager revokes the grant, as it would on a normal return.
  The consumer's next read faults at the buffer, and it continues on the
  quarantine page.
- The producer's next buffer then reaches the consumer intact.

For a CPU client, the revoke does all of the take-back: it unmaps the frame
and returns the capability in one invocation. A device would need its IOTLB
synchronised and its cache maintained between the unmap and the return.

## Not yet covered

- Handoffs are sequential: the producer does not fill the next buffer while
  the consumer reads one.
- A deadline abort for a device that ignores the revoke, with IOTLB and cache
  maintenance. The IOMMU gate is where that can be tested.
- A board's timer: step 4 uses QEMU's goldfish RTC.
- Device DMA. This is a CPU-only path: no IOMMU invalidation or cache
  maintenance is involved, and a single hart means no cross-core TLB shootdown.
