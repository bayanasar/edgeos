/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Messages between the buffer manager (root task) and a client.
 *
 * The client calls the manager; the manager's reply is the next command.
 * Commands carry the buffer's address and length in the client's address
 * space and a sequence number; the client answers with OP_DONE carrying the
 * sequence number and a checksum of what it read back. A client access that
 * faults never answers: the kernel sends the fault to the same endpoint.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

enum {
    OP_READY = 1,
    OP_DONE = 2,

    CMD_FILL = 10,        /* write the pattern for seq, then checksum */
    CMD_CHECK = 11,       /* checksum what is there */
    CMD_PROBE_WRITE = 12, /* write one byte at the address */
    CMD_PROBE_READ = 13,  /* read one byte at the address */
};

/* The pattern a producer writes for a sequence number. */
static inline uint8_t zc_pattern(uint32_t seq, size_t i)
{
    return (uint8_t)(seq * 31u + i * 7u);
}

/* FNV-1a, 32 bit. */
static inline uint32_t zc_fnv1a_step(uint32_t h, uint8_t b)
{
    return (h ^ b) * 16777619u;
}

#define ZC_FNV1A_INIT 2166136261u
