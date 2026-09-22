/*
 * RISC-V IOMMU (spec 1.0) control for one-level device directories and
 * first-stage Sv39 translation. Polled; no interrupts.
 *
 * Whoever holds this state and the register mapping decides what every
 * DMA master behind the IOMMU can reach, so it is part of the TCB.
 */
#pragma once

#include <platsupport/io.h>
#include <stdbool.h>
#include <stdint.h>

#define IOMMU_DOMAIN_MAX_TABLES 8

typedef struct {
    volatile uint8_t *regs;
    ps_dma_man_t *dma;
    uint64_t cap;

    uint64_t *ddt;                 /* one-level device directory */
    uint64_t *cq;                  /* command queue */
    uint32_t cq_tail;
    uint32_t cq_mask;
    uint64_t *fq;                  /* fault queue */
    uint32_t fq_mask;
    volatile uint32_t *fence_word; /* IOFENCE.C completion target */
    uintptr_t fence_pa;
    uint32_t fence_seq;
} riscv_iommu_t;

typedef struct {
    riscv_iommu_t *iommu;
    uint32_t devid;
    uint32_t pscid;
    int ntables;
    uint64_t *table_va[IOMMU_DOMAIN_MAX_TABLES]; /* [0] is the root */
    uintptr_t table_pa[IOMMU_DOMAIN_MAX_TABLES];
} iommu_domain_t;

typedef struct {
    uint32_t cause;
    uint32_t ttype;
    uint32_t devid;
    uint64_t iotval;
} iommu_fault_t;

enum {
    IOMMU_CAUSE_RD_FAULT_S = 13,
    IOMMU_CAUSE_WR_FAULT_S = 15,
    IOMMU_CAUSE_DMA_DISABLED = 256,
    IOMMU_CAUSE_DDT_INVALID = 258,
    IOMMU_CAUSE_MSI_INVALID = 262,
};

/* Queues on, directory off: every inbound DMA is refused and reported. */
int riscv_iommu_init(riscv_iommu_t *m, void *regs, ps_dma_man_t *dma);

/* Switch from Off to a one-level directory. Devices without a valid
 * context remain refused. */
int riscv_iommu_enable_directory(riscv_iommu_t *m);

/* Give devid an empty Sv39 address space: valid context, no mappings. */
int iommu_domain_attach(riscv_iommu_t *m, iommu_domain_t *d, uint32_t devid, uint32_t pscid);

int iommu_map(iommu_domain_t *d, uint64_t iova, uintptr_t pa, bool writable);

/*
 * Enable flat MSI translation for the domain. A device write whose
 * translated address lies in the window {window_pa, file_mask} (the
 * window's page number with the file_mask bits varying) is an MSI; the
 * masked bits select an entry of the MSI page table at msi_pt_pa, and an
 * invalid entry refuses the write.
 */
int iommu_domain_set_msi(iommu_domain_t *d, uintptr_t msi_pt_pa, uintptr_t window_pa,
                         uint64_t file_mask);

/* MSI page-table entry, basic mode: deliver to the interrupt file page. */
uint64_t riscv_iommu_msi_pte(uintptr_t file_pa);

/* Clear the leaf. With invalidate, also flush the IOTLB entry and wait
 * for completion, after which the device can no longer reach the page. */
int iommu_unmap(iommu_domain_t *d, uint64_t iova, bool invalidate);

/* Invalidate one page of the domain's IOTLB entries and wait. */
int iommu_flush(iommu_domain_t *d, uint64_t iova);

/* Returns 1 and fills f if a fault record was pending, 0 if none, -1 on
 * queue overflow or memory fault. */
int riscv_iommu_pop_fault(riscv_iommu_t *m, iommu_fault_t *f);
