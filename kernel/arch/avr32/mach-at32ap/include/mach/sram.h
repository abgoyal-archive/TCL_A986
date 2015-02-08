
#ifndef __ASM_AVR32_ARCH_SRAM_H
#define __ASM_AVR32_ARCH_SRAM_H

#include <linux/genalloc.h>

extern struct gen_pool *sram_pool;

static inline unsigned long sram_alloc(size_t len)
{
	if (!sram_pool)
		return 0UL;

	return gen_pool_alloc(sram_pool, len);
}

static inline void sram_free(unsigned long addr, size_t len)
{
	return gen_pool_free(sram_pool, addr, len);
}

#endif /* __ASM_AVR32_ARCH_SRAM_H */
