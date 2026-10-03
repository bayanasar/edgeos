/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * The untrusted driver: a process of its own that holds the edu device's
 * register page and an endpoint to the IOMMU owner, and nothing else. It
 * can make its device read and write any bus address; only the IOMMU,
 * programmed by the owner, decides what those transfers reach.
 *
 * argv[0]: endpoint slot; argv[1]: virtual address of the register page;
 * argv[2]: CSpace size in bits.
 */
#include <stdlib.h>

#include <sel4/sel4.h>

#include "driver_protocol.h"
#include "edu.h"

/* Lists the capabilities in this CSpace into the message registers. */
static seL4_Word inventory(seL4_Word cspace_bits)
{
    seL4_Word n = 0;
    for (seL4_CPtr slot = 1; slot < (1ul << cspace_bits); slot++) {
        seL4_Word type = seL4_DebugCapIdentify(slot);
        if (type == 0) {
            continue;
        }
        if (n < DRV_INVENTORY_MAX) {
            seL4_Word pa = 0;
            if (type == CAP_TYPE_FRAME) {
                seL4_RISCV_Page_GetAddress_t r = seL4_RISCV_Page_GetAddress(slot);
                pa = r.error ? 0 : r.paddr;
            }
            seL4_SetMR(1 + 3 * n, slot);
            seL4_SetMR(2 + 3 * n, type);
            seL4_SetMR(3 + 3 * n, pa);
        }
        n++;
    }
    seL4_SetMR(0, n);
    return 1 + 3 * (n < DRV_INVENTORY_MAX ? n : DRV_INVENTORY_MAX);
}

int main(int argc, char **argv)
{
    seL4_CPtr ep = (seL4_CPtr)strtoul(argv[0], NULL, 10);
    edu_t dev;
    dev.regs = (volatile uint8_t *)strtoul(argv[1], NULL, 10);
    seL4_Word cspace_bits = strtoul(argv[2], NULL, 10);

    seL4_MessageInfo_t info = seL4_Call(ep, seL4_MessageInfo_new(DRV_READY, 0, 0, 0));
    for (;;) {
        seL4_Word a = seL4_GetMR(0), b = seL4_GetMR(1), len = seL4_GetMR(2);
        seL4_Word words = 1;
        switch (seL4_MessageInfo_get_label(info)) {
        case DRV_TO_BUS:
            seL4_SetMR(0, edu_dma_to_bus(&dev, a, b, len));
            break;
        case DRV_FROM_BUS:
            seL4_SetMR(0, edu_dma_from_bus(&dev, a, b, len));
            break;
        case DRV_INVENTORY:
            words = inventory(cspace_bits);
            break;
        default:
            seL4_SetMR(0, -1);
            break;
        }
        info = seL4_Call(ep, seL4_MessageInfo_new(DRV_DONE, 0, 0, words));
    }
    return 0;
}
