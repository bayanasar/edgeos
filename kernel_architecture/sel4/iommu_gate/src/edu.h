/*
 * QEMU "edu" test device: a PCI bus master with a 4 KiB internal buffer
 * and a single-channel DMA engine. It stands in for an untrusted device.
 * A refused transfer is dropped silently by the device, so the evidence
 * of refusal is the target memory and the IOMMU fault queue.
 */
#pragma once

#include <stdint.h>

#define EDU_BUF_SIZE 4096

typedef struct {
    volatile uint8_t *regs;
} edu_t;

int edu_probe(edu_t *e, void *regs);

/* Device buffer -> bus address (a device write). */
int edu_dma_to_bus(edu_t *e, uint32_t buf_off, uint64_t addr, uint32_t len);

/* Bus address -> device buffer (a device read). */
int edu_dma_from_bus(edu_t *e, uint64_t addr, uint32_t buf_off, uint32_t len);

/* Raise and acknowledge the device interrupt. With MSI enabled, raising
 * makes the device write its MSI message to the bus. */
void edu_raise_irq(edu_t *e);
void edu_ack_irq(edu_t *e);
