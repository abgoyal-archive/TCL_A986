

#include <linux/io.h>
#include <linux/amba/serial.h>
#include <mach/spear.h>

#ifndef __PLAT_UNCOMPRESS_H
#define __PLAT_UNCOMPRESS_H
static inline void putc(int c)
{
	void __iomem *base = (void __iomem *)SPEAR_DBG_UART_BASE;

	while (readl(base + UART01x_FR) & UART01x_FR_TXFF)
		barrier();

	writel(c, base + UART01x_DR);
}

static inline void flush(void)
{
}

#define arch_decomp_setup()
#define arch_decomp_wdog()

#endif /* __PLAT_UNCOMPRESS_H */
