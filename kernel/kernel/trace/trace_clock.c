
#include <linux/spinlock.h>
#include <linux/irqflags.h>
#include <linux/hardirq.h>
#include <linux/module.h>
#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/ktime.h>
#include <linux/trace_clock.h>

#include "trace.h"

u64 notrace trace_clock_local(void)
{
	u64 clock;
	int resched;

	/*
	 * sched_clock() is an architecture implemented, fast, scalable,
	 * lockless clock. It is not guaranteed to be coherent across
	 * CPUs, nor across CPU idle events.
	 */
	resched = ftrace_preempt_disable();
	clock = sched_clock();
	ftrace_preempt_enable(resched);

	return clock;
}

u64 notrace trace_clock(void)
{
	return cpu_clock(raw_smp_processor_id());
}



/* keep prev_time and lock in the same cacheline. */
static struct {
	u64 prev_time;
	arch_spinlock_t lock;
} trace_clock_struct ____cacheline_aligned_in_smp =
	{
		.lock = (arch_spinlock_t)__ARCH_SPIN_LOCK_UNLOCKED,
	};

u64 notrace trace_clock_global(void)
{
	unsigned long flags;
	int this_cpu;
	u64 now;

	local_irq_save(flags);

	this_cpu = raw_smp_processor_id();
	now = cpu_clock(this_cpu);
	/*
	 * If in an NMI context then dont risk lockups and return the
	 * cpu_clock() time:
	 */
	if (unlikely(in_nmi()))
		goto out;

	arch_spin_lock(&trace_clock_struct.lock);

	/*
	 * TODO: if this happens often then maybe we should reset
	 * my_scd->clock to prev_time+1, to make sure
	 * we start ticking with the local clock from now on?
	 */
	if ((s64)(now - trace_clock_struct.prev_time) < 0)
		now = trace_clock_struct.prev_time + 1;

	trace_clock_struct.prev_time = now;

	arch_spin_unlock(&trace_clock_struct.lock);

 out:
	local_irq_restore(flags);

	return now;
}
