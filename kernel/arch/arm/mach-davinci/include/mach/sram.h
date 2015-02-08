
#ifndef __MACH_SRAM_H
#define __MACH_SRAM_H

/* ARBITRARY:  SRAM allocations are multiples of this 2^N size */
#define SRAM_GRANULARITY	512

extern void *sram_alloc(size_t len, dma_addr_t *dma);
extern void sram_free(void *addr, size_t len);

#endif /* __MACH_SRAM_H */
