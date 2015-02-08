
#ifndef OMAP_ARCH_OMAP4_COMMON_H
#define OMAP_ARCH_OMAP4_COMMON_H

#ifdef CONFIG_CACHE_L2X0
extern void __iomem *l2cache_base;
#endif

extern void __iomem *gic_cpu_base_addr;
extern void __iomem *gic_dist_base_addr;

extern void __init gic_init_irq(void);
extern void omap_smc1(u32 fn, u32 arg);

#endif
