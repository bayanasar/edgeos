/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * A client of the buffer manager, in its own address space. It holds no
 * capability to any buffer: the manager maps a buffer in, names it in a
 * command, and takes it away again.
 *
 * argv[0] is the endpoint slot in this process's CSpace.
 */
#include <stdlib.h>

#include <sel4/sel4.h>

#include "protocol.h"

int main(int argc, char **argv)
{
    seL4_CPtr ep = (seL4_CPtr)strtoul(argv[0], NULL, 10);
    seL4_MessageInfo_t info = seL4_Call(ep, seL4_MessageInfo_new(OP_READY, 0, 0, 0));

    for (;;) {
        seL4_Word cmd = seL4_MessageInfo_get_label(info);
        volatile uint8_t *buf = (volatile uint8_t *)seL4_GetMR(0);
        size_t len = seL4_GetMR(1);
        uint32_t seq = seL4_GetMR(2);
        uint32_t sum = ZC_FNV1A_INIT;

        switch (cmd) {
        case CMD_FILL:
            for (size_t i = 0; i < len; i++) {
                buf[i] = zc_pattern(seq, i);
            }
            /* fall through: report what is now there */
        case CMD_CHECK:
            for (size_t i = 0; i < len; i++) {
                sum = zc_fnv1a_step(sum, buf[i]);
            }
            break;
        case CMD_PROBE_WRITE:
            buf[0] = 0x5a;
            break;
        case CMD_PROBE_READ:
            sum = buf[0];
            break;
        default:
            break;
        }
        seL4_SetMR(0, seq);
        seL4_SetMR(1, sum);
        info = seL4_Call(ep, seL4_MessageInfo_new(OP_DONE, 0, 0, 2));
    }
    return 0;
}
