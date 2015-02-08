

#ifndef __ASM_HARDIRQ_H
#define __ASM_HARDIRQ_H

#include <linux/threads.h>
#include <linux/sched.h>
#include <linux/cache.h>
#include <linux/interrupt.h>
#include <asm/lowcore.h>

#define local_softirq_pending() (S390_lowcore.softirq_pending)

#define __ARCH_IRQ_STAT
#define __ARCH_HAS_DO_SOFTIRQ

#define HARDIRQ_BITS	8

void clock_comparator_work(void);

static inline unsigned long long local_tick_disable(void)
{
	unsigned long long old;

	old = S390_lowcore.clock_comparator;
	S390_lowcore.clock_comparator = -1ULL;
	return old;
}

static inline void local_tick_enable(unsigned long long comp)
{
	S390_lowcore.clock_comparator = comp;
}

#endif /* __ASM_HARDIRQ_H */
