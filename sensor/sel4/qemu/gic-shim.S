// SPDX-License-Identifier: BSD-3-Clause
// Stands in for the secure firmware's GIC setup on QEMU's imx8mp-evk, whose
// GIC has no security extensions and resets every interrupt to group 0,
// which seL4 does not take on this platform: put all SPIs and CPU 0's SGIs
// and PPIs in group 1, then enter the Microkit loader at its link address.
    .global _start
_start:
    ldr  x1, =0x38800084        // GICD_IGROUPR1..5: INTIDs 32 to 191
    mov  w2, #-1
    mov  x3, #5
1:  str  w2, [x1], #4
    subs x3, x3, #1
    b.ne 1b
    ldr  x1, =0x38890080        // CPU 0's GICR_IGROUPR0: INTIDs 0 to 31
    str  w2, [x1]
    dsb  sy
    ldr  x16, =0x41000000
    br   x16
