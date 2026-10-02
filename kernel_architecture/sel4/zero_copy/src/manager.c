/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Buffer manager for a zero-copy handoff, step 1: enforced exclusivity
 * towards one client in its own address space.
 *
 * The manager owns the buffer frame and never maps it itself: the contents
 * are the clients' data, never the manager's control state. A grant copies
 * the frame capability with the rights the client is to have and maps the
 * copy into the client's address space. Taking the buffer back revokes the
 * original capability, which deletes the copy and with it the mapping.
 *
 * Every refusal is paired with the same access while granted, which must
 * work, so a probe that could never succeed cannot pass as a refusal.
 * Faulting accesses are completed on a quarantine page, never the buffer.
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
#define BUF_VADDR 0x2000000000ul
#define BUF_LEN   BIT(seL4_PageBits)

#define ALLOCATOR_STATIC_POOL_SIZE (BIT(seL4_PageBits) * 40)
#define ALLOCATOR_VIRTUAL_POOL_SIZE (BIT(seL4_PageBits) * 400)

static char allocator_mem_pool[ALLOCATOR_STATIC_POOL_SIZE];
static sel4utils_alloc_data_t vspace_data;
static simple_t simple;
static vka_t vka;
static vspace_t vspace;

static sel4utils_process_t client;
static seL4_CPtr ep; /* the client's fault endpoint, also the one it calls */
static int checks_run, checks_passed;

typedef struct {
    vka_object_t frame;
    cspacepath_t cap;  /* the manager's original; grants are copies of it */
} frame_t;

typedef struct {
    bool fault;
    seL4_Word fault_addr;
    seL4_Word seq;
    uint32_t sum;
    seL4_Word label;
} answer_t;

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

static frame_t new_frame(void)
{
    frame_t f;
    ZF_LOGF_IF(vka_alloc_frame(&vka, seL4_PageBits, &f.frame), "frame");
    vka_cspace_make_path(&vka, f.frame.cptr, &f.cap);
    return f;
}

/* Maps a frame capability into the client, creating page tables as needed. */
static int map_into_client(seL4_CPtr frame, seL4_CapRights_t rights)
{
    for (int level = 0; level < 4; level++) {
        int err = seL4_RISCV_Page_Map(frame, client.pd.cptr, BUF_VADDR, rights,
                                      seL4_RISCV_Default_VMAttributes);
        if (err != seL4_FailedLookup) {
            return err;
        }
        vka_object_t pt;
        if (vka_alloc_page_table(&vka, &pt)) {
            return -1;
        }
        err = seL4_RISCV_PageTable_Map(pt.cptr, client.pd.cptr, BUF_VADDR,
                                       seL4_RISCV_Default_VMAttributes);
        if (err) {
            return err;
        }
    }
    return -1;
}

/* Copies `f` with `rights` and maps the copy at BUF_VADDR in the client. */
static cspacepath_t grant(frame_t *f, seL4_CapRights_t rights)
{
    cspacepath_t copy;
    ZF_LOGF_IF(vka_cspace_alloc_path(&vka, &copy), "slot");
    ZF_LOGF_IF(vka_cnode_copy(&copy, &f->cap, rights), "copy");
    ZF_LOGF_IF(map_into_client(copy.capPtr, rights), "map grant");
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

/* True once a capability slot no longer holds a frame. */
static bool slot_empty(cspacepath_t *p)
{
    return seL4_RISCV_Page_GetAddress(p->capPtr).error != seL4_NoError;
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

/* Replies to the client's last call with a command and waits for its answer. */
static answer_t command(seL4_Word cmd, seL4_Word seq)
{
    seL4_SetMR(0, BUF_VADDR);
    seL4_SetMR(1, BUF_LEN);
    seL4_SetMR(2, seq);
    seL4_Word badge;
    return decode(seL4_ReplyRecv(ep, seL4_MessageInfo_new(cmd, 0, 0, 3), &badge));
}

/*
 * Completes a faulted access on a quarantine page: maps it where the client
 * faulted, resumes the client, and unmaps it again once the client answers.
 */
static answer_t resume_on_quarantine(frame_t *quarantine)
{
    cspacepath_t q = grant(quarantine, seL4_ReadWrite);
    seL4_Word badge;
    answer_t a = decode(seL4_ReplyRecv(ep, seL4_MessageInfo_new(0, 0, 0, 0), &badge));
    ZF_LOGF_IF(vka_cnode_delete(&q), "unmap quarantine");
    vka_cspace_free_path(&vka, q);
    return a;
}

static uint32_t expected_sum(uint32_t seq)
{
    uint32_t h = ZC_FNV1A_INIT;
    for (size_t i = 0; i < BUF_LEN; i++) {
        h = zc_fnv1a_step(h, zc_pattern(seq, i));
    }
    return h;
}

static void spawn_client(void)
{
    sel4utils_process_config_t config = process_config_default_simple(&simple, "zc_client",
                                                                      seL4_MaxPrio - 1);
    ZF_LOGF_IF(sel4utils_configure_process_custom(&client, &vka, &vspace, config), "client");
    ep = client.fault_endpoint.cptr;
    seL4_CPtr ep_slot = sel4utils_copy_cap_to_process(&client, &vka, ep);

    /* Keep the client's own allocations out of the buffer's 2 MiB. */
    reservation_t r = vspace_reserve_range_at(&client.vspace, (void *)BUF_VADDR, BIT(21),
                                              seL4_AllRights, 1);
    ZF_LOGF_IF(r.res == NULL, "reserve buffer range");

    char strings[1][WORD_STRING_SIZE];
    char *argv[1];
    sel4utils_create_word_args(strings, argv, 1, ep_slot);
    ZF_LOGF_IF(sel4utils_spawn_process_v(&client, &vka, &vspace, 1, argv, 1), "spawn");
}

int main(void)
{
    bootstrap();
    printf("zero-copy: buffer manager on qemu-riscv-virt\n");

    spawn_client();
    seL4_Word badge;
    answer_t a = decode(seL4_Recv(ep, &badge));
    check(!a.fault && a.label == OP_READY, "client starts in its own address space");

    frame_t buf = new_frame();
    frame_t quarantine = new_frame();

    /* Read-write grant: the client's writes land and read back. */
    cspacepath_t g = grant(&buf, seL4_ReadWrite);
    a = command(CMD_FILL, 1);
    check(!a.fault && a.seq == 1 && a.sum == expected_sum(1),
          "a read-write grant carries the client's writes");
    a = command(CMD_PROBE_WRITE, 0);
    check(!a.fault && a.label == OP_DONE, "while granted, the write probe succeeds");
    a = command(CMD_FILL, 2);
    check(!a.fault && a.sum == expected_sum(2), "the client refills the buffer (seq 2)");

    /* Revocation removes the client's copy and its mapping. */
    check(revoke(&buf) == seL4_NoError && slot_empty(&g),
          "revoking the frame deletes the client's copy");
    vka_cspace_free_path(&vka, g);
    a = command(CMD_PROBE_WRITE, 0);
    check(a.fault && a.fault_addr == BUF_VADDR, "after revocation a write faults at the buffer");
    if (a.fault) {
        a = resume_on_quarantine(&quarantine);
        check(!a.fault && a.label == OP_DONE, "the faulted write completes on the quarantine page");
    }
    a = command(CMD_PROBE_READ, 0);
    check(a.fault && a.fault_addr == BUF_VADDR, "after revocation a read faults at the buffer");
    if (a.fault) {
        a = resume_on_quarantine(&quarantine);
    }

    /* Read-only grant of the same frame: the data is intact and unwritable. */
    g = grant(&buf, seL4_CanRead);
    a = command(CMD_CHECK, 2);
    check(!a.fault && a.sum == expected_sum(2),
          "the same frame comes back unchanged: nothing copied it or wrote it");
    a = command(CMD_PROBE_READ, 0);
    check(!a.fault && a.sum == zc_pattern(2, 0), "a read-only grant permits a read");
    a = command(CMD_PROBE_WRITE, 0);
    check(a.fault && a.fault_addr == BUF_VADDR, "a read-only grant refuses a write");
    check(revoke(&buf) == seL4_NoError && slot_empty(&g), "the read-only grant is revoked");
    vka_cspace_free_path(&vka, g);
    if (a.fault) {
        a = resume_on_quarantine(&quarantine);
    }

    /* The faulting writes went to quarantine, never to the buffer. */
    g = grant(&buf, seL4_CanRead);
    a = command(CMD_CHECK, 2);
    check(!a.fault && a.sum == expected_sum(2), "no faulting write reached the buffer");
    revoke(&buf);

    printf("Zero copy: %d/%d checks passed\n", checks_passed, checks_run);
    printf("ZERO_COPY: %s\n", checks_passed == checks_run ? "PASS" : "FAIL");
    seL4_TCB_Suspend(simple_get_tcb(&simple));
    return 0;
}
