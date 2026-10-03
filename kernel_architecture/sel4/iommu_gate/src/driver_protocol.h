/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Messages between the IOMMU owner (root task) and the untrusted driver.
 *
 * The driver calls the owner; the owner's reply is the next command, and
 * the driver answers with DRV_DONE. The commands are what a hostile driver
 * could do with the device it was given: start transfers to any address,
 * and look at the capabilities it holds.
 */
#pragma once

enum {
    DRV_READY = 1,
    DRV_DONE = 2,

    DRV_TO_BUS = 10,    /* MR0 buffer offset, MR1 bus address, MR2 length */
    DRV_FROM_BUS = 11,  /* MR0 bus address, MR1 buffer offset, MR2 length */
    DRV_INVENTORY = 12, /* list every capability in the driver's CSpace */
};

/* DRV_INVENTORY answer: MR0 the number of capabilities, then for each of up
 * to DRV_INVENTORY_MAX: slot, kernel cap type, and physical address if it is
 * a frame (else 0). */
#define DRV_INVENTORY_MAX 12

/* Kernel cap types (include/arch/riscv/arch/64/mode/object/structures.bf). */
enum {
    CAP_TYPE_FRAME = 1,
    CAP_TYPE_UNTYPED = 2,
    CAP_TYPE_ENDPOINT = 4,
};
