

#ifndef __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

#include <linux/io.h>
#include <mach/regs-uart.h>

static void putc(char c)
{
	while (!(__raw_readl(KS8695_UART_PA + KS8695_URLS) & URLS_URTHRE))
		barrier();

	__raw_writel(c, KS8695_UART_PA + KS8695_URTH);
}

static inline void flush(void)
{
	while (!(__raw_readl(KS8695_UART_PA + KS8695_URLS) & URLS_URTE))
		barrier();
}

#define arch_decomp_setup()
#define arch_decomp_wdog()

#endif
