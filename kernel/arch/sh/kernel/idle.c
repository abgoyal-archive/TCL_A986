
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/pm.h>
#include <linux/tick.h>
#include <linux/preempt.h>
#include <linux/thread_info.h>
#include <linux/irqflags.h>
#include <linux/smp.h>
#include <asm/pgalloc.h>
#include <asm/system.h>
#include <asm/atomic.h>
#include <asm/smp.h>

void (*pm_idle)(void) = NULL;

static int hlt_counter;

static int __init nohlt_setup(char *__unused)
{
	hlt_counter = 1;
	return 1;
}
__setup("nohlt", nohlt_setup);

static int __init hlt_setup(char *__unused)
{
	hlt_counter = 0;
	return 1;
}
__setup("hlt", hlt_setup);

static inline int hlt_works(void)
{
	return !hlt_counter;
}

static void poll_idle(void)
{
	local_irq_enable();
	while (!need_resched())
		cpu_relax();
}

void default_idle(void)
{
	if (hlt_works()) {
		clear_thread_flag(TIF_POLLING_NRFLAG);
		smp_mb__after_clear_bit();

		set_bl_bit();
		if (!need_resched()) {
			local_irq_enable();
			cpu_sleep();
		} else
			local_irq_enable();

		set_thread_flag(TIF_POLLING_NRFLAG);
		clear_bl_bit();
	} else
		poll_idle();
}

void cpu_idle(void)
{
	unsigned int cpu = smp_processor_id();

	set_thread_flag(TIF_POLLING_NRFLAG);

	/* endless idle loop with no priority at all */
	while (1) {
		tick_nohz_stop_sched_tick(1);

		while (!need_resched()) {
			check_pgt_cache();
			rmb();

			if (cpu_is_offline(cpu))
				play_dead();

			local_irq_disable();
			/* Don't trace irqs off for idle */
			stop_critical_timings();
			pm_idle();
			/*
			 * Sanity check to ensure that pm_idle() returns
			 * with IRQs enabled
			 */
			WARN_ON(irqs_disabled());
			start_critical_timings();
		}

		tick_nohz_restart_sched_tick();
		preempt_enable_no_resched();
		schedule();
		preempt_disable();
	}
}

void __init select_idle_routine(void)
{
	/*
	 * If a platform has set its own idle routine, leave it alone.
	 */
	if (pm_idle)
		return;

	if (hlt_works())
		pm_idle = default_idle;
	else
		pm_idle = poll_idle;
}

static void do_nothing(void *unused)
{
}

void stop_this_cpu(void *unused)
{
	local_irq_disable();
	set_cpu_online(smp_processor_id(), false);

	for (;;)
		cpu_sleep();
}

void cpu_idle_wait(void)
{
	smp_mb();
	/* kick all the CPUs so that they exit out of pm_idle */
	smp_call_function(do_nothing, NULL, 1);
}
EXPORT_SYMBOL_GPL(cpu_idle_wait);
