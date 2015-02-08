
#ifndef ASMARM_ARCH_SMP_H
#define ASMARM_ARCH_SMP_H

#include <asm/hardware/gic.h>

/* This is required to wakeup the secondary core */
extern void u8500_secondary_startup(void);

#define hard_smp_processor_id()				\
	({						\
		unsigned int cpunum;			\
		__asm__("mrc p15, 0, %0, c0, c0, 5"	\
			: "=r" (cpunum));		\
		cpunum &= 0x0F;				\
	})

static inline void smp_cross_call(const struct cpumask *mask)
{
	gic_raise_softirq(mask, 1);
}
#endif
