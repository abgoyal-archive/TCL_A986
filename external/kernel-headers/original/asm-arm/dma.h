#ifndef __ASM_ARM_DMA_H
#define __ASM_ARM_DMA_H

typedef unsigned int dmach_t;

#include <linux/spinlock.h>
#include <asm/system.h>
#include <asm/scatterlist.h>
#include <asm/arch/dma.h>

#ifndef MAX_DMA_ADDRESS
#define MAX_DMA_ADDRESS	0xffffffff
#endif

typedef unsigned int dmamode_t;

#define DMA_MODE_MASK	3

#define DMA_MODE_READ	 0
#define DMA_MODE_WRITE	 1
#define DMA_MODE_CASCADE 2
#define DMA_AUTOINIT	 4

extern spinlock_t  dma_spin_lock;

static inline unsigned long claim_dma_lock(void)
{
	unsigned long flags;
	spin_lock_irqsave(&dma_spin_lock, flags);
	return flags;
}

static inline void release_dma_lock(unsigned long flags)
{
	spin_unlock_irqrestore(&dma_spin_lock, flags);
}

#define clear_dma_ff(channel)

extern void set_dma_page(dmach_t channel, char pagenr);

extern int  request_dma(dmach_t channel, const char * device_id);

extern void free_dma(dmach_t channel);

extern void enable_dma(dmach_t channel);

extern void disable_dma(dmach_t channel);

extern int dma_channel_active(dmach_t channel);

extern void set_dma_sg(dmach_t channel, struct scatterlist *sg, int nr_sg);

extern void __set_dma_addr(dmach_t channel, void *addr);
#define set_dma_addr(channel, addr)				\
	__set_dma_addr(channel, bus_to_virt(addr))

extern void set_dma_count(dmach_t channel, unsigned long count);

extern void set_dma_mode(dmach_t channel, dmamode_t mode);

extern void set_dma_speed(dmach_t channel, int cycle_ns);

extern int  get_dma_residue(dmach_t channel);

#ifndef NO_DMA
#define NO_DMA	255
#endif

#ifdef CONFIG_PCI
extern int isa_dma_bridge_buggy;
#else
#define isa_dma_bridge_buggy    (0)
#endif

#endif /* _ARM_DMA_H */
