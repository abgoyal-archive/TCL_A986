
#include <linux/bug.h>
#include <linux/clockchips.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/param.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/module.h>

#include <asm/cpu-features.h>
#include <asm/div64.h>
#include <asm/smtc_ipi.h>
#include <asm/time.h>

DEFINE_SPINLOCK(rtc_lock);
EXPORT_SYMBOL(rtc_lock);

int __weak rtc_mips_set_time(unsigned long sec)
{
	return 0;
}

int __weak rtc_mips_set_mmss(unsigned long nowtime)
{
	return rtc_mips_set_time(nowtime);
}

int update_persistent_clock(struct timespec now)
{
	return rtc_mips_set_mmss(now.tv_sec);
}

static int null_perf_irq(void)
{
	return 0;
}

int (*perf_irq)(void) = null_perf_irq;

EXPORT_SYMBOL(perf_irq);


unsigned int mips_hpt_frequency;
EXPORT_SYMBOL(mips_hpt_frequency);

void __init plat_timer_setup(void)
{
	BUG();
}

static __init int cpu_has_mfc0_count_bug(void)
{
	switch (current_cpu_type()) {
	case CPU_R4000PC:
	case CPU_R4000SC:
	case CPU_R4000MC:
		/*
		 * V3.0 is documented as suffering from the mfc0 from count bug.
		 * Afaik this is the last version of the R4000.  Later versions
		 * were marketed as R4400.
		 */
		return 1;

	case CPU_R4400PC:
	case CPU_R4400SC:
	case CPU_R4400MC:
		/*
		 * The published errata for the R4400 upto 3.0 say the CPU
		 * has the mfc0 from count bug.
		 */
		if ((current_cpu_data.processor_id & 0xff) <= 0x30)
			return 1;

		/*
		 * we assume newer revisions are ok
		 */
		return 0;
	}

	return 0;
}

void __init time_init(void)
{
	plat_time_init();

	if (!mips_clockevent_init() || !cpu_has_mfc0_count_bug())
		init_mips_clocksource();
}
