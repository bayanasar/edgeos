#include "pci.h"

#include <stddef.h>

#define PCI_VENDOR_ID   0x00
#define PCI_DEVICE_ID   0x02
#define PCI_COMMAND     0x04
#define PCI_CLASS_REV   0x08
#define PCI_BAR0        0x10
#define PCI_BAR1        0x14
#define PCI_STATUS      0x06
#define PCI_CAP_PTR     0x34

#define PCI_STATUS_CAP_LIST 0x10
#define PCI_CAP_ID_MSI      0x05
#define PCI_MSI_FLAGS       0x02
#define PCI_MSI_ADDR_LO     0x04
#define PCI_MSI_ADDR_HI     0x08
#define PCI_MSI_DATA_64     0x0c
#define PCI_MSI_FLAGS_EN    0x0001
#define PCI_MSI_FLAGS_64BIT 0x0080

#define PCI_COMMAND_MEMORY 0x2
#define PCI_COMMAND_MASTER 0x4

#define PCI_BAR_IO      0x1
#define PCI_BAR_TYPE_64 0x4

static volatile void *cfg(pci_bus_t *bus, int slot, int off)
{
    return bus->ecam + ((size_t)slot << 15) + off;
}

uint16_t pci_read16(pci_bus_t *bus, int slot, int off)
{
    return *(volatile uint16_t *)cfg(bus, slot, off);
}

uint32_t pci_read32(pci_bus_t *bus, int slot, int off)
{
    return *(volatile uint32_t *)cfg(bus, slot, off);
}

void pci_write16(pci_bus_t *bus, int slot, int off, uint16_t val)
{
    *(volatile uint16_t *)cfg(bus, slot, off) = val;
}

void pci_write32(pci_bus_t *bus, int slot, int off, uint32_t val)
{
    *(volatile uint32_t *)cfg(bus, slot, off) = val;
}

int pci_scan(pci_bus_t *bus, pci_dev_t *devs, int max)
{
    int n = 0;
    for (int slot = 0; slot < PCI_SLOTS && n < max; slot++) {
        uint16_t vendor = pci_read16(bus, slot, PCI_VENDOR_ID);
        if (vendor == 0xffff) {
            continue;
        }
        devs[n] = (pci_dev_t) {
            .slot = slot,
            .vendor = vendor,
            .device = pci_read16(bus, slot, PCI_DEVICE_ID),
            .class_rev = pci_read32(bus, slot, PCI_CLASS_REV),
        };
        n++;
    }
    return n;
}

int pci_assign_bar0(pci_bus_t *bus, pci_dev_t *dev)
{
    int slot = dev->slot;
    uint32_t orig = pci_read32(bus, slot, PCI_BAR0);
    if (orig & PCI_BAR_IO) {
        return -1;
    }
    bool is64 = (orig & 0x6) == PCI_BAR_TYPE_64;

    uint16_t cmd = pci_read16(bus, slot, PCI_COMMAND);
    pci_write16(bus, slot, PCI_COMMAND, cmd & ~(PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER));

    pci_write32(bus, slot, PCI_BAR0, 0xffffffff);
    uint64_t mask = pci_read32(bus, slot, PCI_BAR0) & ~0xfull;
    if (is64) {
        pci_write32(bus, slot, PCI_BAR1, 0xffffffff);
        mask |= (uint64_t)pci_read32(bus, slot, PCI_BAR1) << 32;
    } else {
        mask |= 0xffffffff00000000ull;
    }
    uint64_t size = ~mask + 1;
    if (mask == 0xffffffff00000000ull || (size & (size - 1)) != 0) {
        return -1;
    }

    uint64_t addr = (bus->mmio_next + size - 1) & ~(size - 1);
    if (addr + size > bus->mmio_end || (!is64 && addr + size > 0x100000000ull)) {
        return -1;
    }
    bus->mmio_next = addr + size;

    pci_write32(bus, slot, PCI_BAR0, (uint32_t)addr | (orig & 0xf));
    if (is64) {
        pci_write32(bus, slot, PCI_BAR1, (uint32_t)(addr >> 32));
    }
    pci_write16(bus, slot, PCI_COMMAND, (cmd & ~PCI_COMMAND_MASTER) | PCI_COMMAND_MEMORY);

    dev->bar0_paddr = addr;
    dev->bar0_size = size;
    return 0;
}

void pci_set_bus_master(pci_bus_t *bus, const pci_dev_t *dev, bool on)
{
    uint16_t cmd = pci_read16(bus, dev->slot, PCI_COMMAND);
    cmd = on ? (cmd | PCI_COMMAND_MASTER) : (cmd & ~PCI_COMMAND_MASTER);
    pci_write16(bus, dev->slot, PCI_COMMAND, cmd);
}

static int find_cap(pci_bus_t *bus, int slot, uint8_t id)
{
    if (!(pci_read16(bus, slot, PCI_STATUS) & PCI_STATUS_CAP_LIST)) {
        return 0;
    }
    int off = pci_read16(bus, slot, PCI_CAP_PTR) & 0xfc;
    for (int hops = 0; off && hops < 48; hops++) {
        uint16_t hdr = pci_read16(bus, slot, off);
        if ((hdr & 0xff) == id) {
            return off;
        }
        off = (hdr >> 8) & 0xfc;
    }
    return 0;
}

int pci_msi_enable(pci_bus_t *bus, const pci_dev_t *dev, uint64_t addr, uint16_t data)
{
    int cap = find_cap(bus, dev->slot, PCI_CAP_ID_MSI);
    if (cap == 0) {
        return -1;
    }
    uint16_t flags = pci_read16(bus, dev->slot, cap + PCI_MSI_FLAGS);
    if (!(flags & PCI_MSI_FLAGS_64BIT)) {
        return -1;
    }
    /* Single vector, disabled while the message is rewritten. */
    flags &= ~(PCI_MSI_FLAGS_EN | 0x0070);
    pci_write16(bus, dev->slot, cap + PCI_MSI_FLAGS, flags);
    pci_write32(bus, dev->slot, cap + PCI_MSI_ADDR_LO, (uint32_t)addr);
    pci_write32(bus, dev->slot, cap + PCI_MSI_ADDR_HI, (uint32_t)(addr >> 32));
    pci_write16(bus, dev->slot, cap + PCI_MSI_DATA_64, data);
    pci_write16(bus, dev->slot, cap + PCI_MSI_FLAGS, flags | PCI_MSI_FLAGS_EN);
    return 0;
}
