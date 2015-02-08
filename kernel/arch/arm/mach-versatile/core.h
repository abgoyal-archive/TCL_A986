

#ifndef __ASM_ARCH_VERSATILE_H
#define __ASM_ARCH_VERSATILE_H

#include <linux/amba/bus.h>

extern void __init versatile_init(void);
extern void __init versatile_init_irq(void);
extern void __init versatile_map_io(void);
extern struct sys_timer versatile_timer;
extern unsigned int mmc_status(struct device *dev);

#define AMBA_DEVICE(name,busid,base,plat)			\
static struct amba_device name##_device = {			\
	.dev		= {					\
		.coherent_dma_mask = ~0,			\
		.init_name = busid,				\
		.platform_data = plat,				\
	},							\
	.res		= {					\
		.start	= VERSATILE_##base##_BASE,		\
		.end	= (VERSATILE_##base##_BASE) + SZ_4K - 1,\
		.flags	= IORESOURCE_MEM,			\
	},							\
	.dma_mask	= ~0,					\
	.irq		= base##_IRQ,				\
	/* .dma		= base##_DMA,*/				\
}

#endif
