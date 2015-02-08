
#ifndef _ASM_X86_TIMER_H
#define _ASM_X86_TIMER_H
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/percpu.h>
#include <linux/interrupt.h>

#define TICK_SIZE (tick_nsec / 1000)

unsigned long long native_sched_clock(void);
extern int recalibrate_cpu_khz(void);

#if defined(CONFIG_X86_32) && defined(CONFIG_X86_IO_APIC)
extern int timer_ack;
#else
# define timer_ack (0)
#endif

extern int no_timer_check;


DECLARE_PER_CPU(unsigned long, cyc2ns);
DECLARE_PER_CPU(unsigned long long, cyc2ns_offset);

#define CYC2NS_SCALE_FACTOR 10 /* 2^10, carefully chosen */

static inline unsigned long long __cycles_2_ns(unsigned long long cyc)
{
	int cpu = smp_processor_id();
	unsigned long long ns = per_cpu(cyc2ns_offset, cpu);
	ns += cyc * per_cpu(cyc2ns, cpu) >> CYC2NS_SCALE_FACTOR;
	return ns;
}

static inline unsigned long long cycles_2_ns(unsigned long long cyc)
{
	unsigned long long ns;
	unsigned long flags;

	local_irq_save(flags);
	ns = __cycles_2_ns(cyc);
	local_irq_restore(flags);

	return ns;
}

#endif /* _ASM_X86_TIMER_H */
