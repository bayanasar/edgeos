#include "riscv_iommu.h"

#include <stdio.h>
#include <string.h>

/* Register offsets and fields, RISC-V IOMMU spec 1.0 chapter 6. */
#define REG_CAP    0x000
#define REG_DDTP   0x010
#define REG_CQB    0x018
#define REG_CQH    0x020
#define REG_CQT    0x024
#define REG_FQB    0x028
#define REG_FQH    0x030
#define REG_FQT    0x034
#define REG_CQCSR  0x048
#define REG_FQCSR  0x04c

#define CAP_VERSION(c) ((c) & 0xff)
#define CAP_SV39       (1ull << 9)
#define CAP_MSI_FLAT   (1ull << 22)

#define DDTP_MODE_MASK 0xfull
#define DDTP_MODE_OFF  0
#define DDTP_MODE_1LVL 2
#define DDTP_BUSY      (1ull << 4)

#define QUEUE_EN     (1u << 0)
#define QUEUE_MF     (1u << 8)
#define QUEUE_OF     (1u << 9)
#define QUEUE_ON     (1u << 16)
#define QUEUE_BUSY   (1u << 17)
#define CQCSR_CMD_TO  (1u << 9)
#define CQCSR_CMD_ILL (1u << 10)

/* Base format and queue base registers carry PPN in bits 53:10. */
#define PPN_FIELD(pa) (((uint64_t)(pa) >> 12) << 10)
/* ATP registers (fsc, iohgatp) carry PPN in bits 43:0. */
#define ATP_PPN(pa)   ((uint64_t)(pa) >> 12)

#define DC_TC_V         (1ull << 0)
#define DC_FSC_SV39     (8ull << 60)
#define DC_TA_PSCID(p)  ((uint64_t)(p) << 12)
#define DC_WORDS        8     /* extended format, 64 bytes */
#define DC_WORDS_BASE   4     /* base format, 32 bytes */
#define DC_MSIPTP_FLAT  (1ull << 60)

#define MSI_PTE_V       (1ull << 0)
#define MSI_PTE_BASIC   (3ull << 1)

#define CMD_IOTINVAL_VMA  1
#define CMD_IOTINVAL_GVMA (1 | (1 << 7))
#define CMD_IOFENCE_C     2
#define CMD_IODIR_DDT     3
#define CMD_IOTINVAL_AV   (1ull << 10)
#define CMD_IOTINVAL_PSCV (1ull << 32)
#define CMD_IOTINVAL_GV   (1ull << 33)
#define CMD_IOFENCE_AV    (1ull << 10)
#define CMD_IOFENCE_PR    (1ull << 12)
#define CMD_IOFENCE_PW    (1ull << 13)
#define CMD_IODIR_DV      (1ull << 33)

#define PTE_V (1ull << 0)
#define PTE_R (1ull << 1)
#define PTE_W (1ull << 2)
#define PTE_U (1ull << 4)
#define PTE_A (1ull << 6)
#define PTE_D (1ull << 7)

#define PAGE 4096
#define POLL_LIMIT 10000000

static inline void fence(void)
{
    __asm__ volatile("fence iorw, iorw" ::: "memory");
}

static inline uint32_t rd32(riscv_iommu_t *m, int off)
{
    return *(volatile uint32_t *)(m->regs + off);
}

static inline void wr32(riscv_iommu_t *m, int off, uint32_t v)
{
    *(volatile uint32_t *)(m->regs + off) = v;
}

static inline uint64_t rd64(riscv_iommu_t *m, int off)
{
    return *(volatile uint64_t *)(m->regs + off);
}

static inline void wr64(riscv_iommu_t *m, int off, uint64_t v)
{
    *(volatile uint64_t *)(m->regs + off) = v;
}

static void *alloc_page(riscv_iommu_t *m, uintptr_t *pa)
{
    void *va = ps_dma_alloc(m->dma, PAGE, PAGE, 0, PS_MEM_NORMAL);
    if (va == NULL) {
        return NULL;
    }
    memset(va, 0, PAGE);
    *pa = ps_dma_pin(m->dma, va, PAGE);
    return va;
}

static int wait_bits32(riscv_iommu_t *m, int off, uint32_t mask, uint32_t want)
{
    for (int i = 0; i < POLL_LIMIT; i++) {
        if ((rd32(m, off) & mask) == want) {
            return 0;
        }
    }
    return -1;
}

static int cq_submit(riscv_iommu_t *m, uint64_t d0, uint64_t d1)
{
    uint32_t next = (m->cq_tail + 1) & m->cq_mask;
    if (next == (rd32(m, REG_CQH) & m->cq_mask)) {
        return -1; /* full; every submission is followed by a sync */
    }
    m->cq[2 * m->cq_tail] = d0;
    m->cq[2 * m->cq_tail + 1] = d1;
    fence();
    m->cq_tail = next;
    wr32(m, REG_CQT, next);
    uint32_t csr = rd32(m, REG_CQCSR);
    uint32_t err = csr & (QUEUE_MF | CQCSR_CMD_ILL | CQCSR_CMD_TO);
    if (err) {
        printf("iommu: command queue error, cqcsr=%#x\n", csr);
        if (err & CQCSR_CMD_ILL) {
            /* The IOMMU stops at an illegal command without advancing the
             * head, so clearing CMD_ILL alone would execute it again. Replace
             * it with an IOFENCE.C that has no completion action. */
            uint32_t head = rd32(m, REG_CQH) & m->cq_mask;
            m->cq[2 * head] = CMD_IOFENCE_C;
            m->cq[2 * head + 1] = 0;
            fence();
        }
        /* The error bits are write-1-to-clear; left set, they stop the queue
         * and every later submission reports them again. */
        wr32(m, REG_CQCSR, QUEUE_EN | err);
        return -1;
    }
    return 0;
}

/* IOFENCE.C with a completion write: every earlier command has taken
 * effect once the IOMMU has stored the sequence number. PR and PW also
 * commit device reads and writes the IOMMU has already processed, which
 * reclaiming memory from a device requires (spec 1.0, IOFENCE.C note).
 * Requests still in flight before the IOMMU need an interconnect-level
 * flush by whoever drives the device. */
static int cq_sync(riscv_iommu_t *m)
{
    uint32_t seq = ++m->fence_seq;
    *m->fence_word = 0;
    fence();
    uint64_t d0 = CMD_IOFENCE_C | CMD_IOFENCE_AV | CMD_IOFENCE_PR | CMD_IOFENCE_PW |
                  ((uint64_t)seq << 32);
    if (cq_submit(m, d0, m->fence_pa >> 2)) {
        return -1;
    }
    for (int i = 0; i < POLL_LIMIT; i++) {
        if (*m->fence_word == seq) {
            fence();
            return 0;
        }
    }
    printf("iommu: IOFENCE.C did not complete\n");
    return -1;
}

int riscv_iommu_init(riscv_iommu_t *m, void *regs, ps_dma_man_t *dma)
{
    memset(m, 0, sizeof(*m));
    m->regs = regs;
    m->dma = dma;
    m->cap = rd64(m, REG_CAP);

    if (CAP_VERSION(m->cap) != 0x10 || !(m->cap & CAP_SV39)) {
        printf("iommu: unsupported cap %#llx\n", (unsigned long long)m->cap);
        return -1;
    }
    uint64_t ddtp = 0;
    for (int i = 0; i < POLL_LIMIT && ((ddtp = rd64(m, REG_DDTP)) & DDTP_BUSY); i++) {
    }
    if ((ddtp & (DDTP_BUSY | DDTP_MODE_MASK)) != DDTP_MODE_OFF) {
        printf("iommu: directory not Off at reset\n");
        return -1;
    }

    uintptr_t cq_pa, fq_pa, fence_pa;
    m->ddt = alloc_page(m, &m->ddt_pa);
    m->cq = alloc_page(m, &cq_pa);
    m->fq = alloc_page(m, &fq_pa);
    m->fence_word = alloc_page(m, &fence_pa);
    if (!m->ddt || !m->cq || !m->fq || !m->fence_word) {
        return -1;
    }
    m->fence_pa = fence_pa;

    /* One page each: 128 fault records of 32 bytes, 256 commands of 16.
     * LOG2SZ encodes log2(entries) - 1. */
    m->fq_mask = 127;
    wr64(m, REG_FQB, PPN_FIELD(fq_pa) | 6);
    wr32(m, REG_FQH, 0);
    wr32(m, REG_FQCSR, QUEUE_EN);
    if (wait_bits32(m, REG_FQCSR, QUEUE_ON | QUEUE_BUSY, QUEUE_ON)) {
        return -1;
    }

    m->cq_mask = 255;
    wr64(m, REG_CQB, PPN_FIELD(cq_pa) | 7);
    wr32(m, REG_CQT, 0);
    wr32(m, REG_CQCSR, QUEUE_EN);
    if (wait_bits32(m, REG_CQCSR, QUEUE_ON | QUEUE_BUSY, QUEUE_ON)) {
        return -1;
    }
    return 0;
}

int riscv_iommu_enable_directory(riscv_iommu_t *m)
{
    fence();
    /* Root and mode in one store; busy is clear since init. */
    wr64(m, REG_DDTP, PPN_FIELD(m->ddt_pa) | DDTP_MODE_1LVL);
    for (int i = 0; i < POLL_LIMIT; i++) {
        uint64_t v = rd64(m, REG_DDTP);
        if (!(v & DDTP_BUSY)) {
            if ((v & DDTP_MODE_MASK) != DDTP_MODE_1LVL) {
                printf("iommu: one-level directory mode refused\n");
                return -1;
            }
            /* Nothing can be cached yet, but make the rule unconditional. */
            if (cq_submit(m, CMD_IODIR_DDT, 0)) {
                return -1;
            }
            return cq_sync(m);
        }
    }
    return -1;
}

static int dc_words(riscv_iommu_t *m)
{
    return (m->cap & CAP_MSI_FLAT) ? DC_WORDS : DC_WORDS_BASE;
}

int iommu_domain_attach(riscv_iommu_t *m, iommu_domain_t *d, uint32_t devid, uint32_t pscid)
{
    int words = dc_words(m);
    if ((devid + 1) * words * sizeof(uint64_t) > PAGE) {
        return -1; /* beyond a one-level directory */
    }
    memset(d, 0, sizeof(*d));
    d->iommu = m;
    d->devid = devid;
    d->pscid = pscid;
    d->table_va[0] = alloc_page(m, &d->table_pa[0]);
    if (d->table_va[0] == NULL) {
        return -1;
    }
    d->ntables = 1;

    uint64_t *dc = &m->ddt[devid * words];
    dc[1] = 0;                                   /* iohgatp: second stage Bare */
    dc[2] = DC_TA_PSCID(pscid);
    dc[3] = DC_FSC_SV39 | ATP_PPN(d->table_pa[0]);
    for (int i = 4; i < words; i++) {
        dc[i] = 0;                               /* MSI translation off */
    }
    fence();
    dc[0] = DC_TC_V;                             /* publish last */
    fence();
    if (cq_submit(m, CMD_IODIR_DDT | CMD_IODIR_DV | ((uint64_t)devid << 40), 0)) {
        return -1;
    }
    return cq_sync(m);
}

int iommu_domain_set_msi(iommu_domain_t *d, uintptr_t msi_pt_pa, uintptr_t window_pa,
                         uint64_t file_mask)
{
    riscv_iommu_t *m = d->iommu;
    if (!(m->cap & CAP_MSI_FLAT) || (window_pa & (PAGE - 1))) {
        return -1;
    }
    uint64_t *dc = &m->ddt[d->devid * DC_WORDS];
    dc[5] = file_mask;          /* msi_addr_mask, in page-number bits */
    dc[6] = window_pa >> 12;    /* msi_addr_pattern */
    fence();
    dc[4] = DC_MSIPTP_FLAT | ATP_PPN(msi_pt_pa);
    fence();
    /* The context may be cached; it must be reloaded before it counts.
     * MSI page-table entries are cached under the context's GSCID (0,
     * second stage Bare) and are flushed as a whole. */
    if (cq_submit(m, CMD_IODIR_DDT | CMD_IODIR_DV | ((uint64_t)d->devid << 40), 0) ||
        cq_submit(m, CMD_IOTINVAL_GVMA | CMD_IOTINVAL_GV, 0)) {
        return -1;
    }
    return cq_sync(m);
}

uint64_t riscv_iommu_msi_pte(uintptr_t file_pa)
{
    return PPN_FIELD(file_pa) | MSI_PTE_BASIC | MSI_PTE_V;
}

static int iotlb_invalidate(iommu_domain_t *d, uint64_t iova, bool whole_space)
{
    uint64_t d0 = CMD_IOTINVAL_VMA | CMD_IOTINVAL_PSCV | ((uint64_t)d->pscid << 12);
    uint64_t d1 = 0;
    if (!whole_space) {
        d0 |= CMD_IOTINVAL_AV;
        d1 = (iova & ~(uint64_t)(PAGE - 1)) >> 2;
    }
    if (cq_submit(d->iommu, d0, d1)) {
        return -1;
    }
    return cq_sync(d->iommu);
}

static uint64_t *table_for(iommu_domain_t *d, uintptr_t pa)
{
    for (int i = 0; i < d->ntables; i++) {
        if (d->table_pa[i] == pa) {
            return d->table_va[i];
        }
    }
    return NULL;
}

static uint64_t *leaf_slot(iommu_domain_t *d, uint64_t iova, bool alloc)
{
    uint64_t *table = d->table_va[0];
    for (int level = 2; level > 0; level--) {
        uint64_t *pte = &table[(iova >> (12 + 9 * level)) & 0x1ff];
        if (!(*pte & PTE_V)) {
            if (!alloc || d->ntables == IOMMU_DOMAIN_MAX_TABLES) {
                return NULL;
            }
            int i = d->ntables;
            d->table_va[i] = alloc_page(d->iommu, &d->table_pa[i]);
            if (d->table_va[i] == NULL) {
                return NULL;
            }
            d->ntables++;
            fence();
            *pte = PPN_FIELD(d->table_pa[i]) | PTE_V;
        } else if (*pte & (PTE_R | PTE_W)) {
            return NULL; /* superpages are never created here */
        }
        table = table_for(d, (uintptr_t)((*pte >> 10) << 12));
        if (table == NULL) {
            return NULL;
        }
    }
    return &table[(iova >> 12) & 0x1ff];
}

int iommu_map(iommu_domain_t *d, uint64_t iova, uintptr_t pa, bool writable)
{
    if ((iova | pa) & (PAGE - 1) || iova >= (1ull << 38)) {
        return -1;
    }
    int ntables = d->ntables;
    uint64_t *leaf = leaf_slot(d, iova, true);
    if (leaf == NULL || (*leaf & PTE_V)) {
        return -1;
    }
    /* A and D preset: the context does not enable hardware A/D updates. */
    uint64_t pte = PPN_FIELD(pa) | PTE_V | PTE_R | PTE_U | PTE_A;
    if (writable) {
        pte |= PTE_W | PTE_D;
    }
    fence();
    *leaf = pte;
    fence();
    /* Every first-stage change is followed by IOTINVAL.VMA. With ADDR it
     * covers leaf entries only, so new directory levels flush the whole
     * address space. */
    return iotlb_invalidate(d, iova, d->ntables != ntables);
}

int iommu_unmap(iommu_domain_t *d, uint64_t iova, bool invalidate)
{
    uint64_t *leaf = leaf_slot(d, iova, false);
    if (leaf == NULL || !(*leaf & PTE_V)) {
        return -1;
    }
    *leaf = 0;
    fence();
    return invalidate ? iommu_flush(d, iova) : 0;
}

int iommu_flush(iommu_domain_t *d, uint64_t iova)
{
    return iotlb_invalidate(d, iova, false);
}

int riscv_iommu_pop_fault(riscv_iommu_t *m, iommu_fault_t *f)
{
    uint32_t csr = rd32(m, REG_FQCSR);
    if (csr & (QUEUE_MF | QUEUE_OF)) {
        /* Records were lost; report, then re-arm (both bits are W1C). */
        printf("iommu: fault queue error, fqcsr=%#x\n", csr);
        wr32(m, REG_FQCSR, QUEUE_EN | QUEUE_MF | QUEUE_OF);
        return -1;
    }
    uint32_t head = rd32(m, REG_FQH) & m->fq_mask;
    if (head == (rd32(m, REG_FQT) & m->fq_mask)) {
        return 0;
    }
    fence();
    const uint64_t *rec = &m->fq[head * 4];
    f->cause = rec[0] & 0xfff;
    f->ttype = (rec[0] >> 34) & 0x3f;
    f->devid = (uint32_t)(rec[0] >> 40);
    f->iotval = rec[2];
    wr32(m, REG_FQH, (head + 1) & m->fq_mask);
    return 1;
}
