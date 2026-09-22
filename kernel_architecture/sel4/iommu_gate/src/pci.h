/*
 * Minimal PCIe ECAM access for bus 0: enumerate function 0 of each slot,
 * size and place BAR0, and control memory decode and bus mastering.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define PCI_SLOTS 32

typedef struct {
    volatile uint8_t *ecam;  /* bus 0 configuration space, mapped */
    uint64_t mmio_next;      /* next free address in the BAR window */
    uint64_t mmio_end;
} pci_bus_t;

typedef struct {
    uint8_t slot;            /* bus 0, function 0 */
    uint16_t vendor;
    uint16_t device;
    uint32_t class_rev;
    uint64_t bar0_paddr;
    uint64_t bar0_size;
} pci_dev_t;

uint16_t pci_read16(pci_bus_t *bus, int slot, int off);
uint32_t pci_read32(pci_bus_t *bus, int slot, int off);
void pci_write16(pci_bus_t *bus, int slot, int off, uint16_t val);
void pci_write32(pci_bus_t *bus, int slot, int off, uint32_t val);

/* Fill devs with present slots; returns the count. */
int pci_scan(pci_bus_t *bus, pci_dev_t *devs, int max);

/* Size BAR0, place it in the window and enable memory decode. */
int pci_assign_bar0(pci_bus_t *bus, pci_dev_t *dev);

void pci_set_bus_master(pci_bus_t *bus, const pci_dev_t *dev, bool on);

/* Program and enable single-vector MSI; the device must have a 64-bit
 * MSI capability. Returns -1 if it has none. */
int pci_msi_enable(pci_bus_t *bus, const pci_dev_t *dev, uint64_t addr, uint16_t data);

/* Requester ID as the IOMMU sees it: bus 0, function 0. */
static inline uint32_t pci_requester_id(const pci_dev_t *dev)
{
    return (uint32_t)dev->slot << 3;
}
