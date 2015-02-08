

#ifndef __ASM_ARCH_MSM_UNCOMPRESS_H

#include "hardware.h"
#include "linux/io.h"
#include "mach/msm_iomap.h"

static void putc(int c)
{
#if defined(MSM_DEBUG_UART_PHYS)
	unsigned base = MSM_DEBUG_UART_PHYS;
	while (!(readl(base + 0x08) & 0x04)) ;
	writel(c, base + 0x0c);
#endif
}

static inline void flush(void)
{
}

static inline void arch_decomp_setup(void)
{
}

static inline void arch_decomp_wdog(void)
{
}

#endif
