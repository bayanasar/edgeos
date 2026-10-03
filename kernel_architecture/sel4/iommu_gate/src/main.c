/*
 * Root task for the DMA isolation gate on qemu-riscv-virt.
 *
 * One edu device (slot 2) is given three pages through the IOMMU. A
 * second edu device (slot 3) first has no context at all, then a domain
 * of its own. Each check drives a device transfer and judges it by the
 * target memory and the fault queue. The last checks aim the device at the
 * IOMMU's own structures, and give a grant a deadline that a slow device
 * misses.
 */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <allocman/bootstrap.h>
#include <allocman/vka.h>
#include <platsupport/io.h>
#include <sel4/sel4.h>
#include <sel4platsupport/bootinfo.h>
#include <sel4platsupport/io.h>
#include <sel4utils/page_dma.h>
#include <sel4utils/vspace.h>
#include <simple-default/simple-default.h>
#include <utils/util.h>
#include <utils/zf_log_if.h>

#include "edu.h"
#include "pci.h"
#include "riscv_iommu.h"

/* QEMU virt memory map (hw/riscv/virt.c). */
#define ECAM_PADDR      0x30000000ul
#define ECAM_BUS0_SIZE  0x00100000ul
#define PCI_MMIO_BASE   0x40000000ul
#define PCI_MMIO_END    0x80000000ul
/* MSI window: eight 4 KiB interrupt files at the IMSIC S-level base. This
 * machine instantiates no AIA, so nothing decodes these addresses; only
 * the IOMMU's MSI translation can give a write there a destination. */
#define MSI_WINDOW      0x28000000ul
#define MSI_FILE_MASK   0x7ull

#define RISCV_IOMMU_VENDOR 0x1b36
#define RISCV_IOMMU_DEVICE 0x0014
#define EDU_VENDOR         0x1234
#define EDU_DEVICE         0x11e8
#define GRANTED_SLOT       2
#define UNGRANTED_SLOT     3

#define IOVA_GRANT   0x00100000ull  /* read-write */
#define IOVA_RO      0x00101000ull  /* read-only */
#define IOVA_SCRATCH 0x00102000ull  /* read-write, collects exfiltration attempts */
#define IOVA_UNMAPPED 0x00103000ull
#define IOVA_LEASE   0x00104000ull  /* read-write, granted with a deadline */
#define IOVA_MSI     0x00200000ull  /* two pages onto MSI interrupt files 0 and 1 */
#define PSCID_A      1
#define PSCID_B      2
#define XFER         64    /* bytes; keeps refused transfers well inside the fault queue */

/* QEMU virt's goldfish RTC, a nanosecond counter (hw/riscv/virt.c, hw/rtc/goldfish_rtc.c). */
#define RTC_PADDR    0x101000ul
#define RTC_TIME_LOW  (0x00 / 4)
#define RTC_TIME_HIGH (0x04 / 4)
/* edu starts a transfer 100 ms after its command, so a 20 ms deadline is
 * always missed and a 1 s one always met. */
#define DEADLINE_SHORT_NS 20000000ull
#define DEADLINE_LONG_NS  1000000000ull

#define ALLOCATOR_STATIC_POOL_SIZE (BIT(seL4_PageBits) * 40)
#define ALLOCATOR_VIRTUAL_POOL_SIZE (BIT(seL4_PageBits) * 400)

static char allocator_mem_pool[ALLOCATOR_STATIC_POOL_SIZE];
static sel4utils_alloc_data_t vspace_data;
static simple_t simple;
static vka_t vka;
static vspace_t vspace;
static ps_io_ops_t io_ops;
static ps_dma_man_t dma_man;

static riscv_iommu_t iommu;
static volatile uint32_t *rtc;
static int checks_run, checks_passed;

typedef struct {
    uint8_t *va;
    uintptr_t pa;
} page_t;

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

    err = sel4platsupport_new_io_ops(&vspace, &vka, &simple, &io_ops);
    ZF_LOGF_IF(err, "io ops");
    err = sel4utils_new_page_dma_alloc(&vka, &vspace, &dma_man);
    ZF_LOGF_IF(err, "dma allocator");
}

static page_t alloc_page(void)
{
    page_t p;
    p.va = ps_dma_alloc(&dma_man, BIT(seL4_PageBits), BIT(seL4_PageBits), 0, PS_MEM_NORMAL);
    ZF_LOGF_IF(p.va == NULL, "dma page");
    p.pa = ps_dma_pin(&dma_man, p.va, BIT(seL4_PageBits));
    memset(p.va, 0, BIT(seL4_PageBits));
    return p;
}

static void *map_device(uintptr_t paddr, size_t size)
{
    void *va = ps_io_map(&io_ops.io_mapper, paddr, size, 0, PS_MEM_NORMAL);
    ZF_LOGF_IF(va == NULL, "map device %#lx", (unsigned long)paddr);
    return va;
}

static void fill(uint8_t *p, uint8_t seed)
{
    for (int i = 0; i < XFER; i++) {
        p[i] = (uint8_t)(seed + i * 7);
    }
}

static bool holds(const uint8_t *p, uint8_t seed)
{
    for (int i = 0; i < XFER; i++) {
        if (p[i] != (uint8_t)(seed + i * 7)) {
            return false;
        }
    }
    return true;
}

static void check(bool ok, const char *what)
{
    checks_run++;
    if (ok) {
        checks_passed++;
    }
    printf("%s: %s\n", ok ? "PASS" : "FAIL", what);
}

/* Drain the queue; true if it held nothing. */
static bool no_fault(void)
{
    iommu_fault_t f;
    int n = 0, r;
    while ((r = riscv_iommu_pop_fault(&iommu, &f)) == 1) {
        if (n++ == 0) {
            printf("  unexpected fault: cause=%u devid=%#x iotval=%#llx\n", f.cause, f.devid,
                   (unsigned long long)f.iotval);
        }
    }
    return n == 0 && r == 0;
}

/*
 * Drain the queue; true if it held at least one record and every record
 * has the given cause and requester, and an address in [lo, hi) unless
 * lo == hi. QEMU writes several records per refused transfer (16 for 64
 * bytes), because a failed translation covers only the faulting access
 * and the transfer continues piecewise; hardware reports per transaction.
 * Either way no record may disagree.
 */
static bool refused(uint32_t cause, uint32_t devid, uint64_t lo, uint64_t hi)
{
    iommu_fault_t f;
    int n = 0, bad = 0, r;
    while ((r = riscv_iommu_pop_fault(&iommu, &f)) == 1) {
        bool match = f.cause == cause && f.devid == devid &&
                     (lo == hi || (f.iotval >= lo && f.iotval < hi));
        if (n++ == 0 || !match) {
            printf("  fault: cause=%u devid=%#x iotval=%#llx\n", f.cause, f.devid,
                   (unsigned long long)f.iotval);
        }
        bad += !match;
    }
    printf("  %d fault record(s), %d mismatched\n", n, bad);
    return r == 0 && n > 0 && bad == 0;
}

/* A transfer that never completes would make every verdict meaningless. */
static void to_bus(edu_t *e, uint32_t off, uint64_t addr)
{
    ZF_LOGF_IF(edu_dma_to_bus(e, off, addr, XFER), "edu DMA timed out");
}

static void from_bus(edu_t *e, uint64_t addr, uint32_t off)
{
    ZF_LOGF_IF(edu_dma_from_bus(e, addr, off, XFER), "edu DMA timed out");
}

static uint64_t now_ns(void)
{
    uint64_t lo = rtc[RTC_TIME_LOW]; /* latches the high half */
    return (uint64_t)rtc[RTC_TIME_HIGH] << 32 | lo;
}

/* Waits for the device's transfer until the deadline; false if it is still running then. */
static bool done_by(edu_t *e, uint64_t deadline)
{
    while (edu_dma_busy(e)) {
        if (now_ns() >= deadline) {
            return false;
        }
    }
    return true;
}

/* A device write aimed at one of the IOMMU's own structures: refused, and the structure unchanged. */
static bool cannot_write(edu_t *e, uint32_t devid, const void *va, uintptr_t pa)
{
    uint8_t before[XFER];
    memcpy(before, va, XFER);
    to_bus(e, 0, pa);
    return memcmp(before, va, XFER) == 0 && refused(IOMMU_CAUSE_WR_FAULT_S, devid, pa, pa + XFER);
}

static pci_dev_t *find(pci_dev_t *devs, int n, uint16_t vendor, uint16_t device, int slot)
{
    for (int i = 0; i < n; i++) {
        if (devs[i].vendor == vendor && devs[i].device == device &&
            (slot < 0 || devs[i].slot == slot)) {
            return &devs[i];
        }
    }
    return NULL;
}

int main(void)
{
    bootstrap();
    printf("iommu-gate: seL4 root task on qemu-riscv-virt\n");

    pci_bus_t bus = {
        .ecam = map_device(ECAM_PADDR, ECAM_BUS0_SIZE),
        .mmio_next = PCI_MMIO_BASE,
        .mmio_end = PCI_MMIO_END,
    };
    pci_dev_t devs[PCI_SLOTS];
    int n = pci_scan(&bus, devs, PCI_SLOTS);
    for (int i = 0; i < n; i++) {
        printf("pci 00:%02x.0 %04x:%04x class %06x\n", devs[i].slot, devs[i].vendor,
               devs[i].device, devs[i].class_rev >> 8);
    }

    pci_dev_t *iommu_dev = find(devs, n, RISCV_IOMMU_VENDOR, RISCV_IOMMU_DEVICE, -1);
    pci_dev_t *granted = find(devs, n, EDU_VENDOR, EDU_DEVICE, GRANTED_SLOT);
    pci_dev_t *ungranted = find(devs, n, EDU_VENDOR, EDU_DEVICE, UNGRANTED_SLOT);
    ZF_LOGF_IF(!iommu_dev || !granted || !ungranted, "expected IOMMU and two edu devices");

    pci_dev_t *bar_order[] = { iommu_dev, granted, ungranted };
    for (int i = 0; i < 3; i++) {
        ZF_LOGF_IF(pci_assign_bar0(&bus, bar_order[i]), "BAR0 slot %d", bar_order[i]->slot);
        printf("bar0 00:%02x.0 at %#llx size %#llx\n", bar_order[i]->slot,
               (unsigned long long)bar_order[i]->bar0_paddr,
               (unsigned long long)bar_order[i]->bar0_size);
    }

    edu_t dev_a, dev_b;
    ZF_LOGF_IF(edu_probe(&dev_a, map_device(granted->bar0_paddr, granted->bar0_size)), "edu a");
    ZF_LOGF_IF(edu_probe(&dev_b, map_device(ungranted->bar0_paddr, ungranted->bar0_size)), "edu b");
    uint32_t devid_a = pci_requester_id(granted);
    uint32_t devid_b = pci_requester_id(ungranted);

    void *iommu_regs = map_device(iommu_dev->bar0_paddr, iommu_dev->bar0_size);
    pci_set_bus_master(&bus, iommu_dev, true);
    ZF_LOGF_IF(riscv_iommu_init(&iommu, iommu_regs, &dma_man), "iommu init");
    printf("iommu: cap %#llx, queues on, directory Off\n", (unsigned long long)iommu.cap);

    pci_set_bus_master(&bus, granted, true);
    pci_set_bus_master(&bus, ungranted, true);

    page_t grant = alloc_page();    /* mapped read-write at IOVA_GRANT */
    page_t ro = alloc_page();       /* mapped read-only at IOVA_RO */
    page_t scratch = alloc_page();  /* mapped read-write at IOVA_SCRATCH */
    page_t secret = alloc_page();   /* never mapped for any device */
    fill(secret.va, 0x5e);
    printf("pages: grant %#lx ro %#lx scratch %#lx secret %#lx\n", (unsigned long)grant.pa,
           (unsigned long)ro.pa, (unsigned long)scratch.pa, (unsigned long)secret.pa);

    /* 1. Before any directory exists the IOMMU refuses all DMA. */
    fill(grant.va, 0x11);
    to_bus(&dev_a, 0, grant.pa);
    check(holds(grant.va, 0x11) && refused(IOMMU_CAUSE_DMA_DISABLED, devid_a, 0, 0),
          "directory Off refuses device write");

    ZF_LOGF_IF(riscv_iommu_enable_directory(&iommu), "enable directory");

    iommu_domain_t dom;
    ZF_LOGF_IF(iommu_domain_attach(&iommu, &dom, devid_a, PSCID_A), "attach");
    ZF_LOGF_IF(iommu_map(&dom, IOVA_GRANT, grant.pa, true), "map grant");
    ZF_LOGF_IF(iommu_map(&dom, IOVA_RO, ro.pa, false), "map ro");
    ZF_LOGF_IF(iommu_map(&dom, IOVA_SCRATCH, scratch.pa, true), "map scratch");

    /* 2. With the other device's grant live, a device with no context is
     * refused both at that grant's IOVA and at its physical address. */
    to_bus(&dev_b, 0, IOVA_GRANT);
    bool denied = refused(IOMMU_CAUSE_DDT_INVALID, devid_b, 0, 0);
    to_bus(&dev_b, 0, grant.pa);
    check(denied && refused(IOMMU_CAUSE_DDT_INVALID, devid_b, 0, 0) && holds(grant.va, 0x11),
          "device without context refused while another is granted");

    /* 3-4. Permitted read then write through the grant. */
    fill(grant.va, 0x22);
    from_bus(&dev_a, IOVA_GRANT, 0);
    check(no_fault(), "permitted device read");
    memset(grant.va, 0, XFER);
    to_bus(&dev_a, 0, IOVA_GRANT);
    check(holds(grant.va, 0x22) && no_fault(), "permitted device write");

    /* 5. Write aimed at another page's physical address. */
    to_bus(&dev_a, 0, secret.pa);
    check(holds(secret.va, 0x5e) &&
          refused(IOMMU_CAUSE_WR_FAULT_S, devid_a, secret.pa, secret.pa + XFER),
          "write by physical address outside grant refused");

    /* 6. Write just past the mapped range. */
    to_bus(&dev_a, 0, IOVA_UNMAPPED);
    check(refused(IOMMU_CAUSE_WR_FAULT_S, devid_a, IOVA_UNMAPPED, IOVA_UNMAPPED + XFER),
          "write to unmapped IOVA refused");

    /* 7. Read of the secret, then an attempt to write it somewhere visible. */
    from_bus(&dev_a, secret.pa, 1024);
    denied = refused(IOMMU_CAUSE_RD_FAULT_S, devid_a, secret.pa, secret.pa + XFER);
    to_bus(&dev_a, 1024, IOVA_SCRATCH);
    check(denied && !holds(scratch.va, 0x5e) && no_fault(),
          "read outside grant refused, nothing exfiltrated");

    /* 8-9. Read-only mapping: writes refused, reads allowed. */
    fill(ro.va, 0x33);
    to_bus(&dev_a, 0, IOVA_RO);
    check(holds(ro.va, 0x33) &&
          refused(IOMMU_CAUSE_WR_FAULT_S, devid_a, IOVA_RO, IOVA_RO + XFER),
          "write to read-only mapping refused");
    from_bus(&dev_a, IOVA_RO, 2048);
    to_bus(&dev_a, 2048, IOVA_SCRATCH);
    check(holds(scratch.va, 0x33) && no_fault(), "read from read-only mapping permitted");

    /* 10-11. The second device gets its own domain, with its own page at
     * the IOVA the first device uses for its grant. The same IOVA must
     * reach different pages, and the first device's other IOVAs must not
     * exist for the second. */
    page_t own_b = alloc_page();
    iommu_domain_t dom_b;
    ZF_LOGF_IF(iommu_domain_attach(&iommu, &dom_b, devid_b, PSCID_B), "attach b");
    ZF_LOGF_IF(iommu_map(&dom_b, IOVA_GRANT, own_b.pa, true), "map b");
    fill(own_b.va, 0x66);
    from_bus(&dev_b, IOVA_GRANT, 0);
    memset(own_b.va, 0, XFER);
    to_bus(&dev_b, 0, IOVA_GRANT);
    check(holds(own_b.va, 0x66) && holds(grant.va, 0x22) && no_fault(),
          "same IOVA reaches each device's own page");
    to_bus(&dev_b, 0, IOVA_SCRATCH);
    check(holds(scratch.va, 0x33) &&
          refused(IOMMU_CAUSE_WR_FAULT_S, devid_b, IOVA_SCRATCH, IOVA_SCRATCH + XFER),
          "second device cannot reach the first device's mappings");

    /* Informational: clear the leaf without invalidating. The IOTLB is
     * warm from check 4; whether the stale entry is still used is the
     * model's choice, and shows why revocation must include the flush. */
    memset(grant.va, 0, XFER);
    ZF_LOGF_IF(iommu_unmap(&dom, IOVA_GRANT, false), "unmap");
    to_bus(&dev_a, 0, IOVA_GRANT);
    bool stale = holds(grant.va, 0x22);
    no_fault();
    printf("INFO: write after unmap without IOTLB flush %s\n",
           stale ? "landed (stale IOTLB entry)" : "was refused");

    /* 12-13. Revocation: after unmap + flush + fence the grant is gone. */
    ZF_LOGF_IF(iommu_flush(&dom, IOVA_GRANT), "flush");
    memset(grant.va, 0, XFER);
    to_bus(&dev_a, 0, IOVA_GRANT);
    bool untouched = true;
    for (int i = 0; i < XFER; i++) {
        untouched &= grant.va[i] == 0;
    }
    check(untouched && refused(IOMMU_CAUSE_WR_FAULT_S, devid_a, IOVA_GRANT, IOVA_GRANT + XFER),
          "write after revocation refused");

    fill(grant.va, 0x44);
    from_bus(&dev_a, IOVA_GRANT, 3072);
    denied = refused(IOMMU_CAUSE_RD_FAULT_S, devid_a, IOVA_GRANT, IOVA_GRANT + XFER);
    to_bus(&dev_a, 3072, IOVA_SCRATCH);
    check(denied && !holds(scratch.va, 0x44) && no_fault(),
          "read after revocation refused, nothing exfiltrated");

    /* 14-16. MSI remapping. Only interrupt file 0 has an MSI page-table
     * entry; it delivers to a RAM page standing in for the owner's
     * interrupt file, so a delivered message is visible as data. */
    page_t msi_pt = alloc_page();
    page_t file0 = alloc_page();
    ((uint64_t *)msi_pt.va)[0] = riscv_iommu_msi_pte(file0.pa);
    ZF_LOGF_IF(iommu_domain_set_msi(&dom, msi_pt.pa, MSI_WINDOW, MSI_FILE_MASK), "msi table");
    ZF_LOGF_IF(iommu_map(&dom, IOVA_MSI, MSI_WINDOW, true), "map file 0");
    ZF_LOGF_IF(iommu_map(&dom, IOVA_MSI + 0x1000, MSI_WINDOW + 0x1000, true), "map file 1");
    volatile uint32_t *delivered = (volatile uint32_t *)file0.va;

    ZF_LOGF_IF(pci_msi_enable(&bus, granted, IOVA_MSI, 0x2a), "msi");
    edu_raise_irq(&dev_a);
    edu_ack_irq(&dev_a);
    check(*delivered == 0x2a && no_fault(), "MSI to the granted interrupt file delivered");

    ZF_LOGF_IF(pci_msi_enable(&bus, granted, IOVA_MSI + 0x1000, 0x2b), "msi");
    edu_raise_irq(&dev_a);
    edu_ack_irq(&dev_a);
    check(*delivered == 0x2a && refused(IOMMU_CAUSE_MSI_INVALID, devid_a, 0, 0),
          "MSI to an interrupt file without an entry refused");

    ZF_LOGF_IF(pci_msi_enable(&bus, granted, file0.pa, 0x2c), "msi");
    edu_raise_irq(&dev_a);
    edu_ack_irq(&dev_a);
    check(*delivered == 0x2a && refused(IOMMU_CAUSE_WR_FAULT_S, devid_a, file0.pa, file0.pa + 4),
          "MSI aimed at the interrupt file's physical address refused");

    /* 17-20. The IOMMU's own structures. A device that could write its
     * device context, the command queue or its page-table root could give
     * itself any mapping; one that could read its context would learn how
     * it is confined. The device's buffer holds a known pattern first, so a
     * write that landed would show. */
    fill(scratch.va, 0x77);
    from_bus(&dev_a, IOVA_SCRATCH, 0);
    ZF_LOGF_IF(!no_fault(), "pattern load");
    uintptr_t dc_pa;
    uint64_t *dc = riscv_iommu_dc(&iommu, devid_a, &dc_pa);
    check(cannot_write(&dev_a, devid_a, dc, dc_pa), "device write to its own device context refused");
    check(cannot_write(&dev_a, devid_a, iommu.cq, iommu.cq_pa),
          "device write to the IOMMU command queue refused");
    check(cannot_write(&dev_a, devid_a, dom.table_va[0], dom.table_pa[0]),
          "device write to its own page-table root refused");
    from_bus(&dev_a, dc_pa, 1024);
    denied = refused(IOMMU_CAUSE_RD_FAULT_S, devid_a, dc_pa, dc_pa + XFER);
    memset(scratch.va, 0, XFER);
    to_bus(&dev_a, 1024, IOVA_SCRATCH);
    check(denied && memcmp(scratch.va, dc, XFER) != 0 && no_fault(),
          "device read of its own device context refused, nothing exfiltrated");

    /* 21-25. Grant deadlines. Every grant carries a deadline, and expiry
     * takes the page back without the device's cooperation: unmap, IOTLB
     * invalidation and fence, then the page goes to its next owner. QEMU
     * models no caches, so there is no cache maintenance to do. edu performs
     * a started transfer regardless, as a device that ignores the revoke. */
    rtc = map_device(RTC_PADDR, BIT(seL4_PageBits));
    page_t lease = alloc_page();

    ZF_LOGF_IF(iommu_map(&dom, IOVA_LEASE, lease.pa, true), "map lease");
    ZF_LOGF_IF(edu_start_to_bus(&dev_a, 0, IOVA_LEASE, XFER), "start");
    bool in_time = done_by(&dev_a, now_ns() + DEADLINE_LONG_NS);
    check(in_time && holds(lease.va, 0x77) && no_fault(),
          "a device that finishes before its deadline: its write lands");
    ZF_LOGF_IF(iommu_unmap(&dom, IOVA_LEASE, true), "return lease");

    /* The device uses the grant once, so its translation is cached, then
     * starts a transfer it will not finish before the deadline. */
    ZF_LOGF_IF(iommu_map(&dom, IOVA_LEASE, lease.pa, true), "map lease");
    to_bus(&dev_a, 0, IOVA_LEASE);
    ZF_LOGF_IF(!no_fault(), "warm-up");
    memset(lease.va, 0, XFER);
    ZF_LOGF_IF(edu_start_to_bus(&dev_a, 0, IOVA_LEASE, XFER), "start");
    check(!done_by(&dev_a, now_ns() + DEADLINE_SHORT_NS),
          "a device still busy at its deadline is not waited for");
    ZF_LOGF_IF(iommu_unmap(&dom, IOVA_LEASE, true), "abort");
    check(edu_dma_busy(&dev_a), "the abort completes while the device's transfer is pending");
    fill(lease.va, 0x99); /* the next owner's data */
    ZF_LOGF_IF(edu_dma_wait(&dev_a), "late transfer");
    check(holds(lease.va, 0x99) &&
          refused(IOMMU_CAUSE_WR_FAULT_S, devid_a, IOVA_LEASE, IOVA_LEASE + XFER),
          "the device's late write is refused: the next owner's data is intact");

    ZF_LOGF_IF(iommu_map(&dom, IOVA_LEASE, lease.pa, true), "map lease");
    from_bus(&dev_a, IOVA_LEASE, 512);
    ZF_LOGF_IF(!no_fault(), "warm-up");
    memset(lease.va, 0, XFER);
    ZF_LOGF_IF(edu_start_from_bus(&dev_a, IOVA_LEASE, 512, XFER), "start");
    bool late = !done_by(&dev_a, now_ns() + DEADLINE_SHORT_NS);
    ZF_LOGF_IF(iommu_unmap(&dom, IOVA_LEASE, true), "abort");
    bool pending = edu_dma_busy(&dev_a);
    fill(lease.va, 0xab); /* the next owner's data */
    ZF_LOGF_IF(edu_dma_wait(&dev_a), "late transfer");
    denied = refused(IOMMU_CAUSE_RD_FAULT_S, devid_a, IOVA_LEASE, IOVA_LEASE + XFER);
    memset(scratch.va, 0, XFER);
    to_bus(&dev_a, 512, IOVA_SCRATCH);
    check(late && pending && denied && !holds(scratch.va, 0xab) && no_fault(),
          "the device's late read is refused: nothing of the next owner's data leaks");

    printf("IOMMU gate: %d/%d checks passed\n", checks_passed, checks_run);
    printf("IOMMU_GATE: %s\n", checks_run > 0 && checks_passed == checks_run ? "PASS" : "FAIL");

    seL4_TCB_Suspend(seL4_CapInitThreadTCB);
    return 0;
}
