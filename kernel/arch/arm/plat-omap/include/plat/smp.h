
#ifndef OMAP_ARCH_SMP_H
#define OMAP_ARCH_SMP_H

#include <asm/hardware/gic.h>

#define set_event()	__asm__ __volatile__ ("sev" : : : "memory")

/* Needed for secondary core boot */
extern void omap_secondary_startup(void);
extern u32 omap_modify_auxcoreboot0(u32 set_mask, u32 clear_mask);
extern void omap_auxcoreboot_addr(u32 cpu_addr);

static inline void smp_cross_call(const struct cpumask *mask)
{
	gic_raise_softirq(mask, 1);
}

#define hard_smp_processor_id()			\
	({						\
		unsigned int cpunum;			\
		__asm__("mrc p15, 0, %0, c0, c0, 5"	\
			: "=r" (cpunum));		\
		cpunum &= 0x0F;				\
	})

#endif
