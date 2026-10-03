/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Buffer manager for a zero-copy handoff on seL4.
 *
 * The manager owns the buffer frames and never maps one itself: the
 * contents are the clients' data, never the manager's control state. A
 * grant copies a frame capability with the rights the client is to have
 * and maps the copy into the client's address space. Taking a buffer back
 * revokes the original capability, which deletes every copy and with it
 * every mapping. The original is revocable because it came from
 * Untyped_Retype; a copy of a frame capability is not (on RISC-V,
 * Arch_isCapRevocable is false), so only the original's holder can take a
 * buffer back.
 *
 * Step 1 checks one client alone. Step 2 adds a consumer and passes a
 * stream of buffers from producer to consumer through a ring. Every
 * refusal is paired with the same access while granted, which must work,
 * and the refusals are made while another client holds the buffer.
 * Faulting accesses are completed on a quarantine page, never a buffer.
 *
 * Step 3 counts what a handoff costs, from the kernel's own log of its
 * entries, against two baselines: a trusted copier, and one mapping shared
 * for good with no enforcement.
 *
 * Step 4 gives a grant a deadline on a hardware timer and revokes on expiry,
 * without the client's cooperation.
 */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <allocman/bootstrap.h>
#include <allocman/vka.h>
#include <platsupport/io.h>
#include <sel4/benchmark_track_types.h>
#include <sel4/sel4.h>
#include <sel4platsupport/bootinfo.h>
#include <sel4platsupport/io.h>
#include <sel4utils/process.h>
#include <sel4utils/vspace.h>
#include <simple-default/simple-default.h>
#include <utils/util.h>
#include <vka/capops.h>

#include "protocol.h"

/* A top-level page-table entry (1 GiB under Sv39) that nothing else uses. */
#define BUF_VADDR  0x2000000000ul
#define BUF_LEN    BIT(seL4_PageBits)
#define RING       4   /* buffers in the ring */
#define HANDOFFS   32  /* producer-to-consumer handoffs through the ring */

/* Step 3 uses the next two 2 MiB regions; clients reserve all three. */
#define COST_VADDR  (BUF_VADDR + BIT(21))      /* buffers of 4 KiB frames */
#define LARGE_VADDR (BUF_VADDR + 2 * BIT(21))  /* a buffer of one 2 MiB frame */
#define CLIENT_RESERVED BIT(23)
#define COST_HANDOFFS 8
#define MAX_FRAMES 16

/* Step 4: QEMU virt's goldfish RTC, a nanosecond counter with one alarm. */
#define RTC_PADDR   0x101000
#define RTC_IRQ     11
#define TIMER_BADGE 1  /* client endpoints are unbadged */
#define DEADLINE_NS 100000000ull

#define ALLOCATOR_STATIC_POOL_SIZE (BIT(seL4_PageBits) * 40)
#define ALLOCATOR_VIRTUAL_POOL_SIZE (BIT(seL4_PageBits) * 400)

static char allocator_mem_pool[ALLOCATOR_STATIC_POOL_SIZE];
static sel4utils_alloc_data_t vspace_data;
static simple_t simple;
static vka_t vka;
static vspace_t vspace;
static int checks_run, checks_passed;

typedef struct {
    sel4utils_process_t proc;
    seL4_CPtr ep;        /* its fault endpoint, also the one it calls */
    cspacepath_t reply;  /* saved reply capability for its last call or fault */
} client_t;

typedef struct {
    vka_object_t frame;
    cspacepath_t cap;  /* the manager's original; grants are copies of it */
} frame_t;

typedef struct {
    bool fault;
    seL4_Word fault_addr;
    seL4_Word label;
    seL4_Word seq;
    uint32_t sum;
} answer_t;

static frame_t quarantine;

static void bootstrap(void)
{
    simple_default_init_bootinfo(&simple, platsupport_get_bootinfo());
    allocman_t *allocman = bootstrap_use_current_simple(&simple, ALLOCATOR_STATIC_POOL_SIZE,
                                                        allocator_mem_pool);
    ZF_LOGF_IF(allocman == NULL, "allocman");
    allocman_make_vka(&vka, allocman);

    int err = sel4utils_bootstrap_vspace_with_bootinfo_leaky(&vspace, &vspace_data,
                                                             simple_get_pd(&simple), &vka,
                                                             platsupport_get_bootinfo());
    ZF_LOGF_IF(err, "vspace");

    void *vaddr;
    reservation_t res = vspace_reserve_range(&vspace, ALLOCATOR_VIRTUAL_POOL_SIZE,
                                             seL4_AllRights, 1, &vaddr);
    ZF_LOGF_IF(res.res == NULL, "virtual pool");
    bootstrap_configure_virtual_pool(allocman, vaddr, ALLOCATOR_VIRTUAL_POOL_SIZE,
                                     simple_get_pd(&simple));
}

static void check(bool ok, const char *what)
{
    checks_run++;
    if (ok) {
        checks_passed++;
    }
    printf("%s: %s\n", ok ? "PASS" : "FAIL", what);
}

static seL4_Word slot_va(int slot)
{
    return BUF_VADDR + (seL4_Word)slot * BUF_LEN;
}

static frame_t new_frame(seL4_Word bits)
{
    frame_t f;
    ZF_LOGF_IF(vka_alloc_frame(&vka, bits, &f.frame), "frame");
    vka_cspace_make_path(&vka, f.frame.cptr, &f.cap);
    return f;
}

/*
 * Maps a frame capability into a client, creating page tables as needed.
 * The page tables are never freed: bounded here because addresses are reused,
 * but a long-running manager would have to reclaim them.
 */
static int map_into(client_t *c, seL4_CPtr frame, seL4_Word va, seL4_CapRights_t rights)
{
    for (int level = 0; level < 4; level++) {
        int err = seL4_RISCV_Page_Map(frame, c->proc.pd.cptr, va, rights,
                                      seL4_RISCV_Default_VMAttributes);
        if (err != seL4_FailedLookup) {
            return err;
        }
        vka_object_t pt;
        if (vka_alloc_page_table(&vka, &pt)) {
            return -1;
        }
        err = seL4_RISCV_PageTable_Map(pt.cptr, c->proc.pd.cptr, va,
                                       seL4_RISCV_Default_VMAttributes);
        if (err) {
            return err;
        }
    }
    return -1;
}

/* Copies `f` with `rights` into the empty slot `copy` and maps it at `va` in client `c`. */
static void grant_into(cspacepath_t *copy, frame_t *f, client_t *c, seL4_Word va,
                       seL4_CapRights_t rights)
{
    ZF_LOGF_IF(vka_cnode_copy(copy, &f->cap, rights), "copy");
    ZF_LOGF_IF(map_into(c, copy->capPtr, va, rights), "map grant");
}

/* As grant_into, in a new slot. */
static cspacepath_t grant(frame_t *f, client_t *c, seL4_Word va, seL4_CapRights_t rights)
{
    cspacepath_t copy;
    ZF_LOGF_IF(vka_cspace_alloc_path(&vka, &copy), "slot");
    grant_into(&copy, f, c, va, rights);
    return copy;
}

/* Takes every grant of `f` back. Returns the revoke's error code. */
static int revoke(frame_t *f)
{
#ifdef ZC_MUTANT_SKIP_REVOKE
    return 0;
#else
    return vka_cnode_revoke(&f->cap);
#endif
}

/* Revokes `f` and releases the slot of the copy `g`, which the revoke emptied. */
static bool take_back(frame_t *f, cspacepath_t *g)
{
    bool ok = revoke(f) == seL4_NoError
              && seL4_RISCV_Page_GetAddress(g->capPtr).error != seL4_NoError;
    vka_cspace_free_path(&vka, *g);
    return ok;
}

static answer_t decode(seL4_MessageInfo_t info)
{
    answer_t a = { .label = seL4_MessageInfo_get_label(info) };
    if (a.label == seL4_Fault_VMFault) {
        a.fault = true;
        a.fault_addr = seL4_GetMR(seL4_VMFault_Addr);
    } else if (a.label == OP_DONE) {
        a.seq = seL4_GetMR(0);
        a.sum = seL4_GetMR(1);
    }
    return a;
}

/* Waits for the client's next call or fault and keeps the reply capability. */
static answer_t wait_for(client_t *c)
{
    seL4_Word badge;
    seL4_MessageInfo_t info = seL4_Recv(c->ep, &badge);
    answer_t a = decode(info);
    ZF_LOGF_IF(vka_cnode_saveCaller(&c->reply), "save caller");
    return a;
}

/* Replies to the client's last call with a command. */
static void send_command(client_t *c, seL4_Word cmd, seL4_Word va, size_t len, seL4_Word seq)
{
    seL4_SetMR(0, va);
    seL4_SetMR(1, len);
    seL4_SetMR(2, seq);
    seL4_Send(c->reply.capPtr, seL4_MessageInfo_new(cmd, 0, 0, 3));
}

/* Sends a command and waits for the client's answer. */
static answer_t command_len(client_t *c, seL4_Word cmd, seL4_Word va, size_t len, seL4_Word seq)
{
    send_command(c, cmd, va, len, seq);
    return wait_for(c);
}

static answer_t command(client_t *c, seL4_Word cmd, seL4_Word va, seL4_Word seq)
{
    return command_len(c, cmd, va, BUF_LEN, seq);
}

/*
 * Completes a faulted access on the quarantine page: maps it where the
 * client faulted, resumes the client, and unmaps it once the client answers.
 * This is the test harness's policy, so that one client can be probed many
 * times. A real manager would stop or restart a client that faults, not hand
 * it a writable page to finish the access.
 */
static answer_t resume_on_quarantine(client_t *c, seL4_Word fault_addr)
{
    cspacepath_t q = grant(&quarantine, c, fault_addr & ~(BUF_LEN - 1), seL4_ReadWrite);
    seL4_Send(c->reply.capPtr, seL4_MessageInfo_new(0, 0, 0, 0));
    answer_t a = wait_for(c);
    ZF_LOGF_IF(vka_cnode_delete(&q), "unmap quarantine");
    vka_cspace_free_path(&vka, q);
    return a;
}

/* True if the access faulted at `va`; the client is then resumed harmlessly. */
static bool refused(client_t *c, answer_t a, seL4_Word va)
{
    if (!a.fault) {
        return false;
    }
    bool at_va = a.fault_addr == va;
    answer_t r = resume_on_quarantine(c, a.fault_addr);
    return at_va && !r.fault && r.label == OP_DONE;
}

static uint32_t expected_sum_len(uint32_t seq, size_t len)
{
    uint32_t h = ZC_FNV1A_INIT;
    for (size_t i = 0; i < len; i++) {
        h = zc_fnv1a_step(h, zc_pattern(seq, i));
    }
    return h;
}

static uint32_t expected_sum(uint32_t seq)
{
    return expected_sum_len(seq, BUF_LEN);
}

static void spawn(client_t *c)
{
    sel4utils_process_config_t config = process_config_default_simple(&simple, "zc_client",
                                                                      seL4_MaxPrio - 1);
    ZF_LOGF_IF(sel4utils_configure_process_custom(&c->proc, &vka, &vspace, config), "client");
    c->ep = c->proc.fault_endpoint.cptr;
    seL4_CPtr ep_slot = sel4utils_copy_cap_to_process(&c->proc, &vka, c->ep);
    ZF_LOGF_IF(vka_cspace_alloc_path(&vka, &c->reply), "reply slot");

    /* Keep the client's own allocations out of the buffers' regions. */
    reservation_t r = vspace_reserve_range_at(&c->proc.vspace, (void *)BUF_VADDR,
                                              CLIENT_RESERVED, seL4_AllRights, 1);
    ZF_LOGF_IF(r.res == NULL, "reserve buffer range");

    char strings[1][WORD_STRING_SIZE];
    char *argv[1];
    sel4utils_create_word_args(strings, argv, 1, ep_slot);
    ZF_LOGF_IF(sel4utils_spawn_process_v(&c->proc, &vka, &vspace, 1, argv, 1), "spawn");
}

/* Step 1: one client, one buffer. */
static void single_client(client_t *p, frame_t *buf)
{
    seL4_Word va = slot_va(0);
    answer_t a;

    cspacepath_t g = grant(buf, p, va, seL4_ReadWrite);
    a = command(p, CMD_FILL, va, 1);
    check(!a.fault && a.seq == 1 && a.sum == expected_sum(1),
          "a read-write grant carries the client's writes");
    a = command(p, CMD_PROBE_WRITE, va, 0);
    check(!a.fault && a.label == OP_DONE, "while granted, the write probe succeeds");
    a = command(p, CMD_FILL, va, 2);
    check(!a.fault && a.sum == expected_sum(2), "the client refills the buffer (seq 2)");

    /* A copy is not revocable: revoking one deletes nothing derived from it. */
    cspacepath_t g2;
    ZF_LOGF_IF(vka_cspace_alloc_path(&vka, &g2), "slot");
    ZF_LOGF_IF(vka_cnode_copy(&g2, &g, seL4_AllRights), "copy of the copy");
    bool kept = vka_cnode_revoke(&g) == seL4_NoError
                && seL4_RISCV_Page_GetAddress(g2.capPtr).error == seL4_NoError;
    a = command(p, CMD_PROBE_READ, va, 0);
    check(kept && !a.fault && a.sum == zc_pattern(2, 0),
          "revoking the client's copy takes nothing back: its copy and mapping remain");

    check(take_back(buf, &g), "revoking the frame deletes the client's copy");
    check(seL4_RISCV_Page_GetAddress(g2.capPtr).error != seL4_NoError,
          "revoking the frame also deletes a copy of the copy");
    vka_cspace_free_path(&vka, g2);
    check(refused(p, command(p, CMD_PROBE_WRITE, va, 0), va),
          "after revocation a write faults at the buffer, and completes on quarantine");
    check(refused(p, command(p, CMD_PROBE_READ, va, 0), va),
          "after revocation a read faults at the buffer");

    g = grant(buf, p, va, seL4_CanRead);
    a = command(p, CMD_CHECK, va, 2);
    check(!a.fault && a.sum == expected_sum(2),
          "the same frame comes back unchanged: nothing copied it or wrote it");
    a = command(p, CMD_PROBE_READ, va, 0);
    check(!a.fault && a.sum == zc_pattern(2, 0), "a read-only grant permits a read");
    a = command(p, CMD_PROBE_WRITE, va, 0);
    /* The read-only mapping must go before the quarantine page can take its place. */
    check(take_back(buf, &g), "the read-only grant is revoked");
    check(refused(p, a, va), "a read-only grant refuses a write");

    g = grant(buf, p, va, seL4_CanRead);
    a = command(p, CMD_CHECK, va, 2);
    check(!a.fault && a.sum == expected_sum(2), "no faulting write reached the buffer");
    take_back(buf, &g);
}

/* Step 2: producer to consumer through a ring of buffers. */
static void producer_consumer(client_t *p, client_t *c, frame_t ring[RING])
{
    int intact = 0;
    for (uint32_t n = 1; n <= HANDOFFS; n++) {
        int s = n % RING;
        seL4_Word va = slot_va(s);

        cspacepath_t g = grant(&ring[s], p, va, seL4_ReadWrite);
        answer_t made = command(p, CMD_FILL, va, n);
        bool back = take_back(&ring[s], &g);

        g = grant(&ring[s], c, va, seL4_CanRead);
        answer_t seen = command(c, CMD_CHECK, va, n);
        back = take_back(&ring[s], &g) && back;

        if (!made.fault && !seen.fault && back && made.sum == expected_sum(n)
            && seen.sum == made.sum) {
            intact++;
        }
    }
    printf("handoffs intact: %d/%d\n", intact, HANDOFFS);
    check(intact == HANDOFFS, "every handoff through the ring arrives intact");

    /* Exclusivity while the other side holds the buffer. */
    seL4_Word va = slot_va(0);
    answer_t a;
    cspacepath_t g = grant(&ring[0], p, va, seL4_ReadWrite);
    a = command(p, CMD_FILL, va, 100);
    check(!a.fault && a.sum == expected_sum(100), "the producer fills buffer 0 (seq 100)");
    a = command(p, CMD_PROBE_WRITE, va, 0);
    check(!a.fault && a.label == OP_DONE, "while the producer holds it, its write succeeds");
    a = command(p, CMD_FILL, va, 100);
    take_back(&ring[0], &g);

    cspacepath_t gc = grant(&ring[0], c, va, seL4_CanRead);
    check(refused(p, command(p, CMD_PROBE_WRITE, va, 0), va),
          "while the consumer holds it, the producer's write faults");
    check(refused(p, command(p, CMD_PROBE_READ, va, 0), va),
          "while the consumer holds it, the producer's read faults");
    a = command(c, CMD_CHECK, va, 100);
    check(!a.fault && a.sum == expected_sum(100),
          "the consumer reads what the producer wrote, untouched by its refused write");
    check(refused(c, command(c, CMD_PROBE_READ, slot_va(1), 0), slot_va(1)),
          "the consumer cannot read a buffer it does not hold");
    a = command(c, CMD_PROBE_WRITE, va, 0);
    take_back(&ring[0], &gc);
    check(refused(c, a, va), "the consumer cannot write the buffer it holds");

    gc = grant(&ring[0], c, va, seL4_CanRead);
    a = command(c, CMD_CHECK, va, 100);
    check(!a.fault && a.sum == expected_sum(100), "buffer 0 is intact after every refusal");
    take_back(&ring[0], &gc);
}

/*
 * Step 3: what a handoff costs, as the kernel logs it.
 *
 * Three schemes move the same buffer from producer to consumer:
 * - enforced: the step 2 handoff, a grant and a revoke on each side;
 * - copier: each client keeps a private buffer for good, and the manager
 *   copies the producer's into the consumer's;
 * - shared: one buffer mapped for good, read-write in the producer and
 *   read-only in the consumer. Nothing stops the producer writing while the
 *   consumer reads, so the consumer trusts the producer: a floor for the cost.
 * The kernel logs every entry (KernelBenchmarks track_kernel_entries)
 * between a log reset and a finalize; the manager counts the bytes it copies.
 * QEMU's timing is not hardware timing, so only counts are reported.
 *
 * A grant reuses the slot its last revoke emptied, as a long-running manager
 * would. Allocating and freeing a slot per grant would also log a
 * seL4_DebugCapIdentify per free, which vka_cspace_free makes in debug builds.
 */

/* Cap types as the kernel logs them (include/arch/riscv/arch/64/mode/object/structures.bf). */
enum { CAP_FRAME = 1, CAP_ENDPOINT = 4, CAP_REPLY = 8, CAP_CNODE = 10 };

/* The kernel logs minus the syscall number. */
#define SYS(n) ((seL4_Word)-(n))

typedef enum { ENFORCED, COPIER, SHARED, SCHEMES } scheme_t;
static const char *const scheme_name[SCHEMES] = { "enforced", "copier", "shared" };

typedef struct {
    int frames;
    seL4_Word bits;
    frame_t f[MAX_FRAMES];
} buffer_t;

typedef struct {
    seL4_Word ipc;     /* calls and receives on endpoints, sends on reply caps */
    seL4_Word save;    /* CNode_SaveCaller */
    seL4_Word copy;    /* CNode_Copy */
    seL4_Word map;     /* Page_Map */
    seL4_Word revoke;  /* CNode_Revoke */
    seL4_Word other;   /* any other system call */
    seL4_Word reset;   /* benchmark and debug calls: only the reset that opens the window */
    seL4_Word irq;     /* unlabelled entries: interrupts (see tally_entry) */
    size_t copied;     /* bytes the manager copied */
} tally_t;

static benchmark_track_kernel_entry_t *kernel_log;

static void log_init(void)
{
    kernel_log = vspace_new_pages(&vspace, seL4_AllRights, 1, seL4_LargePageBits);
    ZF_LOGF_IF(kernel_log == NULL, "log buffer");
    ZF_LOGF_IF(seL4_BenchmarkSetLogBuffer(vspace_get_cap(&vspace, kernel_log)) != seL4_NoError,
               "set log buffer");
}

static void tally_entry(tally_t *t, kernel_entry_t e)
{
    /*
     * The RISC-V kernel labels system calls only: its interrupt and exception
     * entries set no path and are logged as Entry_Unknown (ARM and x86 label
     * them). In a window these are timer interrupts, since a client fault
     * would also reach the manager as a fault answer, which fails the run.
     */
    switch (e.path) {
    case Entry_Syscall:
        break;
    case Entry_UnknownSyscall:
        t->reset++;
        return;
    default:
        t->irq++;
        return;
    }
    seL4_Word sys = e.syscall_no;
    if (e.cap_type == CAP_ENDPOINT && (sys == SYS(seL4_SysCall) || sys == SYS(seL4_SysRecv))) {
        t->ipc++;
    } else if (e.cap_type == CAP_REPLY && sys == SYS(seL4_SysSend)) {
        t->ipc++;
    } else if (e.cap_type == CAP_CNODE && sys == SYS(seL4_SysCall)
               && e.invocation_tag == CNodeSaveCaller) {
        t->save++;
    } else if (e.cap_type == CAP_CNODE && sys == SYS(seL4_SysCall)
               && e.invocation_tag == CNodeCopy) {
        t->copy++;
    } else if (e.cap_type == CAP_CNODE && sys == SYS(seL4_SysCall)
               && e.invocation_tag == CNodeRevoke) {
        t->revoke++;
    } else if (e.cap_type == CAP_FRAME && sys == SYS(seL4_SysCall)
               && e.invocation_tag == RISCVPageMap) {
        t->map++;
    } else {
        t->other++;
    }
}

static size_t buffer_len(const buffer_t *b)
{
    return (size_t)b->frames << b->bits;
}

static buffer_t new_buffer(int frames, seL4_Word bits)
{
    buffer_t b = { .frames = frames, .bits = bits };
    for (int i = 0; i < frames; i++) {
        b.f[i] = new_frame(bits);
    }
    return b;
}

/* Revokes every grant of `b` and frees its frames. */
static void free_buffer(buffer_t *b)
{
    for (int i = 0; i < b->frames; i++) {
        ZF_LOGF_IF(vka_cnode_revoke(&b->f[i].cap), "revoke");
        vka_free_object(&vka, &b->f[i].frame);
    }
}

/* Grants every frame of `b` to `c` from `va` on, into the empty slots `g`. */
static void grant_buffer(buffer_t *b, client_t *c, seL4_Word va, seL4_CapRights_t rights,
                         cspacepath_t g[])
{
    for (int i = 0; i < b->frames; i++) {
        grant_into(&g[i], &b->f[i], c, va + ((seL4_Word)i << b->bits), rights);
    }
}

/* Takes every frame of `b` back; the revokes empty the slots in `g` for reuse. */
static void revoke_buffer(buffer_t *b)
{
    for (int i = 0; i < b->frames; i++) {
        revoke(&b->f[i]);
    }
}

static void alloc_paths(int n, cspacepath_t g[])
{
    for (int i = 0; i < n; i++) {
        ZF_LOGF_IF(vka_cspace_alloc_path(&vka, &g[i]), "slot");
    }
}

/* Releases slots that a revoke has emptied. */
static void free_paths(int n, cspacepath_t g[])
{
    for (int i = 0; i < n; i++) {
        vka_cspace_free_path(&vka, g[i]);
    }
}

/* For the copier: maps copies of `b`'s frames into the manager itself. */
static uint8_t *map_into_manager(buffer_t *b, cspacepath_t g[])
{
    void *va;
    seL4_CPtr caps[MAX_FRAMES];
    reservation_t res = vspace_reserve_range_aligned(&vspace, buffer_len(b), b->bits,
                                                     seL4_ReadWrite, 1, &va);
    ZF_LOGF_IF(res.res == NULL, "manager range");
    for (int i = 0; i < b->frames; i++) {
        ZF_LOGF_IF(vka_cnode_copy(&g[i], &b->f[i].cap, seL4_ReadWrite), "copy");
        caps[i] = g[i].capPtr;
    }
    ZF_LOGF_IF(vspace_map_pages_at_vaddr(&vspace, caps, NULL, va, b->frames, b->bits, res),
               "map into manager");
    return va;
}

/*
 * The range stays reserved: a range that held 4 KiB frames keeps its page
 * table, and a 2 MiB frame cannot be mapped over one.
 */
static void unmap_from_manager(buffer_t *b, uint8_t *va)
{
    vspace_unmap_pages(&vspace, va, b->frames, b->bits, VSPACE_PRESERVE);
}

/*
 * Runs COST_HANDOFFS handoffs of one buffer under one scheme inside a log
 * window, after one handoff outside it that creates any page tables the
 * clients still lack. Sets `intact` if every consumer checksum equals the
 * producer's and the expected pattern.
 */
static tally_t run(scheme_t s, client_t *p, client_t *c, int frames, seL4_Word bits, bool *intact)
{
    buffer_t buf = new_buffer(frames, bits), priv = { 0 };
    size_t len = buffer_len(&buf);
    seL4_Word va = bits == seL4_PageBits ? COST_VADDR : LARGE_VADDR;
    cspacepath_t gp[MAX_FRAMES], gc[MAX_FRAMES], gm[MAX_FRAMES], gn[MAX_FRAMES];
    uint8_t *from = NULL, *to = NULL;
    answer_t made[COST_HANDOFFS + 1], seen[COST_HANDOFFS + 1];
    tally_t t = { 0 };

    alloc_paths(frames, gp);
    alloc_paths(frames, gc);
    if (s == COPIER) {
        alloc_paths(frames, gm);
        alloc_paths(frames, gn);
        priv = new_buffer(frames, bits);
        grant_buffer(&buf, p, va, seL4_ReadWrite, gp);
        grant_buffer(&priv, c, va, seL4_CanRead, gc);
        from = map_into_manager(&buf, gm);
        to = map_into_manager(&priv, gn);
    } else if (s == SHARED) {
        grant_buffer(&buf, p, va, seL4_ReadWrite, gp);
        grant_buffer(&buf, c, va, seL4_CanRead, gc);
    }

    for (uint32_t k = 0; k <= COST_HANDOFFS; k++) {
        if (k == 1) {
            seL4_BenchmarkResetLog();
        }
        if (s == ENFORCED) {
            grant_buffer(&buf, p, va, seL4_ReadWrite, gp);
        }
        made[k] = command_len(p, CMD_FILL, va, len, 1000 + k);
        if (s == ENFORCED) {
            revoke_buffer(&buf);
            grant_buffer(&buf, c, va, seL4_CanRead, gc);
        } else if (s == COPIER) {
            memcpy(to, from, len);
            if (k > 0) {
                t.copied += len;
            }
        }
        seen[k] = command_len(c, CMD_CHECK, va, len, 1000 + k);
        if (s == ENFORCED) {
            revoke_buffer(&buf);
        }
    }
    seL4_Word entries = seL4_BenchmarkFinalizeLog();

    for (seL4_Word i = 0; i < entries; i++) {
        tally_entry(&t, kernel_log[i].entry);
    }
    *intact = true;
    for (uint32_t k = 0; k <= COST_HANDOFFS; k++) {
        *intact = *intact && !made[k].fault && !seen[k].fault
                  && made[k].sum == expected_sum_len(1000 + k, len) && seen[k].sum == made[k].sum;
    }

    if (s == COPIER) {
        unmap_from_manager(&buf, from);
        unmap_from_manager(&priv, to);
        free_buffer(&priv);
    }
    free_buffer(&buf); /* its revokes empty every copy still held */
    if (s == COPIER) {
        free_paths(frames, gm);
        free_paths(frames, gn);
    }
    free_paths(frames, gp);
    free_paths(frames, gc);
    return t;
}

static void cost(client_t *p, client_t *c)
{
    static const struct {
        int frames;
        seL4_Word bits;
    } sizes[] = {
        { 1, seL4_PageBits },
        { MAX_FRAMES, seL4_PageBits },
        { 1, seL4_LargePageBits },
    };
    const seL4_Word h = COST_HANDOFFS;
    bool intact[SCHEMES] = { true, true, true };
    bool clean = true, ipc = true, cap_ops = true, copied = true;

    log_init();
    for (scheme_t s = ENFORCED; s < SCHEMES; s++) {
        for (size_t z = 0; z < ARRAY_SIZE(sizes); z++) {
            bool ok;
            tally_t t = run(s, p, c, sizes[z].frames, sizes[z].bits, &ok);
            size_t len = (size_t)sizes[z].frames << sizes[z].bits;
            seL4_Word ops = s == ENFORCED ? 2 * (seL4_Word)sizes[z].frames * h : 0;

            printf("cost: %s, %zu bytes in %d frame(s), %lu handoffs: ipc %lu, save-caller %lu, "
                   "copy %lu, map %lu, revoke %lu, other %lu, reset %lu, copied %zu bytes; "
                   "%lu interrupts\n",
                   scheme_name[s], len, sizes[z].frames, h, t.ipc, t.save, t.copy, t.map,
                   t.revoke, t.other, t.reset, t.copied, t.irq);
            intact[s] = intact[s] && ok;
            clean = clean && t.other == 0 && t.reset == 1;
            ipc = ipc && t.ipc == 6 * h && t.save == 2 * h;
            cap_ops = cap_ops && t.copy == ops && t.map == ops && t.revoke == ops;
            copied = copied && t.copied == (s == COPIER ? len * h : 0);
        }
    }
    check(intact[ENFORCED], "enforced: every handoff arrives intact at 4 KiB, 64 KiB and 2 MiB");
    check(intact[COPIER], "copier: every handoff arrives intact at every size");
    check(intact[SHARED], "shared: every handoff arrives intact at every size");
    check(clean, "interrupts aside, the kernel logged nothing the protocol does not account for");
    check(ipc, "every scheme makes 3 IPC entries and 1 save-caller per command");
    check(cap_ops,
          "only the enforced scheme copies, maps and revokes: once per frame on each side");
    check(copied, "only the copier copies data: the buffer's length per handoff");
}

/*
 * Step 4: a grant with a deadline.
 *
 * Every grant carries a deadline, and expiry takes the buffer back without
 * the client's cooperation. For a CPU client the take-back is the revoke,
 * which unmaps and returns the capability in one invocation; a device would
 * also need its IOTLB synchronised and the cache maintained in between.
 * The timer is QEMU virt's goldfish RTC: a nanosecond counter whose alarm
 * raises interrupt 11, delivered to a notification bound to the manager, so
 * a wait for a client also ends at the deadline.
 */

/* Goldfish RTC registers, as 32-bit word indices (QEMU hw/rtc/goldfish_rtc.c). */
enum {
    RTC_TIME_LOW = 0x00 / 4,
    RTC_TIME_HIGH = 0x04 / 4,
    RTC_ALARM_LOW = 0x08 / 4,
    RTC_ALARM_HIGH = 0x0c / 4,
    RTC_IRQ_ENABLED = 0x10 / 4,
    RTC_CLEAR_ALARM = 0x14 / 4,
    RTC_CLEAR_INTERRUPT = 0x1c / 4,
};

static volatile uint32_t *rtc;
static seL4_CPtr timer_ntfn, timer_irq;

static void timer_init(void)
{
    ps_io_ops_t io_ops;
    ZF_LOGF_IF(sel4platsupport_new_io_ops(&vspace, &vka, &simple, &io_ops), "io ops");
    rtc = ps_io_map(&io_ops.io_mapper, RTC_PADDR, BIT(seL4_PageBits), 0, PS_MEM_NORMAL);
    ZF_LOGF_IF(rtc == NULL, "map RTC");

    vka_object_t ntfn;
    cspacepath_t ntfn_path, badged, irq;
    ZF_LOGF_IF(vka_alloc_notification(&vka, &ntfn), "notification");
    vka_cspace_make_path(&vka, ntfn.cptr, &ntfn_path);
    ZF_LOGF_IF(vka_cspace_alloc_path(&vka, &badged), "slot");
    ZF_LOGF_IF(vka_cnode_mint(&badged, &ntfn_path, seL4_AllRights, TIMER_BADGE), "badge");
    ZF_LOGF_IF(vka_cspace_alloc_path(&vka, &irq), "slot");
    ZF_LOGF_IF(seL4_IRQControl_Get(seL4_CapIRQControl, RTC_IRQ, irq.root, irq.capPtr, irq.capDepth),
               "RTC interrupt");
    ZF_LOGF_IF(seL4_IRQHandler_SetNotification(irq.capPtr, badged.capPtr), "RTC notification");
    ZF_LOGF_IF(seL4_TCB_BindNotification(simple_get_tcb(&simple), ntfn.cptr), "bind");
    timer_ntfn = ntfn.cptr;
    timer_irq = irq.capPtr;
    rtc[RTC_IRQ_ENABLED] = 1;
}

static uint64_t now_ns(void)
{
    uint64_t lo = rtc[RTC_TIME_LOW]; /* latches the high half */
    return (uint64_t)rtc[RTC_TIME_HIGH] << 32 | lo;
}

/* Clears the alarm and any interrupt it raised, and drops a signal still pending. */
static void disarm(void)
{
    rtc[RTC_CLEAR_ALARM] = 1;
    rtc[RTC_CLEAR_INTERRUPT] = 1;
    seL4_Word pending = 0;
    seL4_Poll(timer_ntfn, &pending);
    if (pending & TIMER_BADGE) {
        seL4_IRQHandler_Ack(timer_irq);
    }
}

/*
 * Waits for client `c`'s next call or fault until `deadline`. Sets `expired`,
 * and returns no answer, if the alarm comes first.
 */
static answer_t wait_until(client_t *c, uint64_t deadline, bool *expired)
{
    rtc[RTC_ALARM_HIGH] = deadline >> 32;
    rtc[RTC_ALARM_LOW] = (uint32_t)deadline; /* arms the alarm */
    seL4_Word badge;
    seL4_MessageInfo_t info = seL4_Recv(c->ep, &badge);
    *expired = badge & TIMER_BADGE;
    if (*expired) {
        rtc[RTC_CLEAR_INTERRUPT] = 1;
        seL4_IRQHandler_Ack(timer_irq);
        return (answer_t){ 0 };
    }
    answer_t a = decode(info); /* before any other system call overwrites the message */
    ZF_LOGF_IF(vka_cnode_saveCaller(&c->reply), "save caller");
    disarm();
    return a;
}

/* Fills `buf` in the producer and leaves it with nobody. */
static bool produce(client_t *p, frame_t *buf, seL4_Word va, uint32_t seq)
{
    cspacepath_t g = grant(buf, p, va, seL4_ReadWrite);
    answer_t a = command(p, CMD_FILL, va, seq);
    return take_back(buf, &g) && !a.fault && a.sum == expected_sum(seq);
}

static void deadlines(client_t *p, client_t *c)
{
    /*
     * The quarantine page holds only zeros and the 0x5a of refused write
     * probes, and CMD_HOLD answers once the byte it reads changes, so the
     * buffer's first byte must be neither: zc_pattern(300, 0) is 0x54.
     */
    const uint32_t seq = 300;
    seL4_Word va = slot_va(0);
    frame_t buf = new_frame(seL4_PageBits);
    bool expired, ok;
    answer_t a;

    timer_init();
    ok = produce(p, &buf, va, seq);

    cspacepath_t g = grant(&buf, c, va, seL4_CanRead);
    send_command(c, CMD_CHECK, va, BUF_LEN, seq);
    a = wait_until(c, now_ns() + DEADLINE_NS, &expired);
    ok = take_back(&buf, &g) && ok;
    check(ok && !expired && !a.fault && a.sum == expected_sum(seq),
          "a consumer that answers before its deadline finishes normally");

    g = grant(&buf, c, va, seL4_CanRead);
    uint64_t deadline = now_ns() + DEADLINE_NS;
    send_command(c, CMD_HOLD, va, BUF_LEN, seq);
    a = wait_until(c, deadline, &expired);
    uint64_t woke = now_ns();
    check(expired && woke >= deadline,
          "a consumer that holds the buffer is cut off at its deadline, not before");
    check(take_back(&buf, &g), "the abort revokes without the consumer's answer: its copy is gone");
    a = wait_until(c, now_ns() + DEADLINE_NS, &expired);
    check(!expired && a.fault && a.fault_addr == va,
          "the consumer, still reading, faults at the buffer once it is revoked");
    answer_t r = expired ? (answer_t){ 0 } : resume_on_quarantine(c, a.fault_addr);
    check(!r.fault && r.label == OP_DONE, "resumed on quarantine, the consumer answers again");

    ok = produce(p, &buf, va, seq + 1);
    g = grant(&buf, c, va, seL4_CanRead);
    a = command(c, CMD_CHECK, va, seq + 1);
    ok = take_back(&buf, &g) && ok;
    check(ok && !a.fault && a.sum == expected_sum(seq + 1),
          "after the abort the stream goes on: the next handoff arrives intact");
}

int main(void)
{
    bootstrap();
    printf("zero-copy: buffer manager on qemu-riscv-virt\n");
    quarantine = new_frame(seL4_PageBits);

    client_t producer, consumer;
    spawn(&producer);
    answer_t a = wait_for(&producer);
    check(!a.fault && a.label == OP_READY, "the producer starts in its own address space");

    frame_t buf = new_frame(seL4_PageBits);
    single_client(&producer, &buf);

    spawn(&consumer);
    a = wait_for(&consumer);
    check(!a.fault && a.label == OP_READY, "the consumer starts in its own address space");

    frame_t ring[RING];
    for (int i = 0; i < RING; i++) {
        ring[i] = new_frame(seL4_PageBits);
    }
    producer_consumer(&producer, &consumer, ring);
    cost(&producer, &consumer);
    deadlines(&producer, &consumer);

    printf("Zero copy: %d/%d checks passed\n", checks_passed, checks_run);
    printf("ZERO_COPY: %s\n", checks_passed == checks_run ? "PASS" : "FAIL");
    seL4_TCB_Suspend(simple_get_tcb(&simple));
    return 0;
}
