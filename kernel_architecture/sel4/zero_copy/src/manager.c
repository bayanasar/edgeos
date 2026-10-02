/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Buffer manager for a zero-copy handoff on seL4.
 *
 * The manager owns the buffer frames and never maps one itself: the
 * contents are the clients' data, never the manager's control state. A
 * grant copies a frame capability with the rights the client is to have
 * and maps the copy into the client's address space. Taking a buffer back
 * revokes the original capability, which deletes every copy and with it
 * every mapping.
 *
 * Step 1 checks one client alone. Step 2 adds a consumer and passes a
 * stream of buffers from producer to consumer through a ring. Every
 * refusal is paired with the same access while granted, which must work,
 * and the refusals are made while another client holds the buffer.
 * Faulting accesses are completed on a quarantine page, never a buffer.
 */
#include <stdbool.h>
#include <stdio.h>

#include <allocman/bootstrap.h>
#include <allocman/vka.h>
#include <sel4/sel4.h>
#include <sel4platsupport/bootinfo.h>
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

static frame_t new_frame(void)
{
    frame_t f;
    ZF_LOGF_IF(vka_alloc_frame(&vka, seL4_PageBits, &f.frame), "frame");
    vka_cspace_make_path(&vka, f.frame.cptr, &f.cap);
    return f;
}

/* Maps a frame capability into a client, creating page tables as needed. */
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

/* Copies `f` with `rights` and maps the copy at `va` in client `c`. */
static cspacepath_t grant(frame_t *f, client_t *c, seL4_Word va, seL4_CapRights_t rights)
{
    cspacepath_t copy;
    ZF_LOGF_IF(vka_cspace_alloc_path(&vka, &copy), "slot");
    ZF_LOGF_IF(vka_cnode_copy(&copy, &f->cap, rights), "copy");
    ZF_LOGF_IF(map_into(c, copy.capPtr, va, rights), "map grant");
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

/* Replies to the client's last call with a command and waits for its answer. */
static answer_t command(client_t *c, seL4_Word cmd, seL4_Word va, seL4_Word seq)
{
    seL4_SetMR(0, va);
    seL4_SetMR(1, BUF_LEN);
    seL4_SetMR(2, seq);
    seL4_Send(c->reply.capPtr, seL4_MessageInfo_new(cmd, 0, 0, 3));
    return wait_for(c);
}

/*
 * Completes a faulted access on the quarantine page: maps it where the
 * client faulted, resumes the client, and unmaps it once the client answers.
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

static uint32_t expected_sum(uint32_t seq)
{
    uint32_t h = ZC_FNV1A_INIT;
    for (size_t i = 0; i < BUF_LEN; i++) {
        h = zc_fnv1a_step(h, zc_pattern(seq, i));
    }
    return h;
}

static void spawn(client_t *c)
{
    sel4utils_process_config_t config = process_config_default_simple(&simple, "zc_client",
                                                                      seL4_MaxPrio - 1);
    ZF_LOGF_IF(sel4utils_configure_process_custom(&c->proc, &vka, &vspace, config), "client");
    c->ep = c->proc.fault_endpoint.cptr;
    seL4_CPtr ep_slot = sel4utils_copy_cap_to_process(&c->proc, &vka, c->ep);
    ZF_LOGF_IF(vka_cspace_alloc_path(&vka, &c->reply), "reply slot");

    /* Keep the client's own allocations out of the buffers' 2 MiB. */
    reservation_t r = vspace_reserve_range_at(&c->proc.vspace, (void *)BUF_VADDR, BIT(21),
                                              seL4_AllRights, 1);
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

    check(take_back(buf, &g), "revoking the frame deletes the client's copy");
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

int main(void)
{
    bootstrap();
    printf("zero-copy: buffer manager on qemu-riscv-virt\n");
    quarantine = new_frame();

    client_t producer, consumer;
    spawn(&producer);
    answer_t a = wait_for(&producer);
    check(!a.fault && a.label == OP_READY, "the producer starts in its own address space");

    frame_t buf = new_frame();
    single_client(&producer, &buf);

    spawn(&consumer);
    a = wait_for(&consumer);
    check(!a.fault && a.label == OP_READY, "the consumer starts in its own address space");

    frame_t ring[RING];
    for (int i = 0; i < RING; i++) {
        ring[i] = new_frame();
    }
    producer_consumer(&producer, &consumer, ring);

    printf("Zero copy: %d/%d checks passed\n", checks_passed, checks_run);
    printf("ZERO_COPY: %s\n", checks_passed == checks_run ? "PASS" : "FAIL");
    seL4_TCB_Suspend(simple_get_tcb(&simple));
    return 0;
}
