#include "edu.h"

#define EDU_ID       0x00
#define EDU_LIVENESS 0x04
#define EDU_IRQ_RAISE 0x60
#define EDU_IRQ_ACK  0x64
#define EDU_DMA_SRC  0x80
#define EDU_DMA_DST  0x88
#define EDU_DMA_CNT  0x90
#define EDU_DMA_CMD  0x98

#define EDU_ID_VALUE    0x010000edu
#define EDU_BUF_BASE    0x40000
#define EDU_CMD_RUN     0x1
#define EDU_CMD_TO_BUS  0x2

#define POLL_LIMIT 20000000

static inline uint32_t rd32(edu_t *e, int off)
{
    return *(volatile uint32_t *)(e->regs + off);
}

static inline void wr32(edu_t *e, int off, uint32_t v)
{
    *(volatile uint32_t *)(e->regs + off) = v;
}

static inline uint64_t rd64(edu_t *e, int off)
{
    return *(volatile uint64_t *)(e->regs + off);
}

static inline void wr64(edu_t *e, int off, uint64_t v)
{
    *(volatile uint64_t *)(e->regs + off) = v;
}

int edu_probe(edu_t *e, void *regs)
{
    e->regs = regs;
    if (rd32(e, EDU_ID) != EDU_ID_VALUE) {
        return -1;
    }
    wr32(e, EDU_LIVENESS, 0x5a5a1234);
    return rd32(e, EDU_LIVENESS) == ~0x5a5a1234u ? 0 : -1;
}

static int dma(edu_t *e, uint64_t src, uint64_t dst, uint32_t len, uint64_t dir)
{
    if (rd64(e, EDU_DMA_CMD) & EDU_CMD_RUN) {
        return -1;
    }
    __asm__ volatile("fence iorw, iorw" ::: "memory");
    wr64(e, EDU_DMA_SRC, src);
    wr64(e, EDU_DMA_DST, dst);
    wr64(e, EDU_DMA_CNT, len);
    wr64(e, EDU_DMA_CMD, EDU_CMD_RUN | dir);
    for (int i = 0; i < POLL_LIMIT; i++) {
        if (!(rd64(e, EDU_DMA_CMD) & EDU_CMD_RUN)) {
            __asm__ volatile("fence iorw, iorw" ::: "memory");
            return 0;
        }
    }
    return -1;
}

int edu_dma_to_bus(edu_t *e, uint32_t buf_off, uint64_t addr, uint32_t len)
{
    if (buf_off + len > EDU_BUF_SIZE) {
        return -1;
    }
    return dma(e, EDU_BUF_BASE + buf_off, addr, len, EDU_CMD_TO_BUS);
}

int edu_dma_from_bus(edu_t *e, uint64_t addr, uint32_t buf_off, uint32_t len)
{
    if (buf_off + len > EDU_BUF_SIZE) {
        return -1;
    }
    return dma(e, addr, EDU_BUF_BASE + buf_off, len, 0);
}

void edu_raise_irq(edu_t *e)
{
    wr32(e, EDU_IRQ_RAISE, 1);
}

void edu_ack_irq(edu_t *e)
{
    wr32(e, EDU_IRQ_ACK, 1);
}
