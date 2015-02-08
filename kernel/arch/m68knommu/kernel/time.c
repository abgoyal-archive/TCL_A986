

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/profile.h>
#include <linux/time.h>
#include <linux/timex.h>

#include <asm/machdep.h>
#include <asm/irq_regs.h>

#define	TICK_SIZE (tick_nsec / 1000)

static inline int set_rtc_mmss(unsigned long nowtime)
{
	if (mach_set_clock_mmss)
		return mach_set_clock_mmss (nowtime);
	return -1;
}

#ifndef CONFIG_GENERIC_CLOCKEVENTS
irqreturn_t arch_timer_interrupt(int irq, void *dummy)
{

	if (current->pid)
		profile_tick(CPU_PROFILING);

	write_seqlock(&xtime_lock);

	do_timer(1);

	write_sequnlock(&xtime_lock);

#ifndef CONFIG_SMP
	update_process_times(user_mode(get_irq_regs()));
#endif
	return(IRQ_HANDLED);
}
#endif

static unsigned long read_rtc_mmss(void)
{
	unsigned int year, mon, day, hour, min, sec;

	if (mach_gettod)
		mach_gettod(&year, &mon, &day, &hour, &min, &sec);
	else
		year = mon = day = hour = min = sec = 0;

	if ((year += 1900) < 1970)
		year += 100;

	return  mktime(year, mon, day, hour, min, sec);
}

void read_persistent_clock(struct timespec *ts)
{
	ts->tv_sec = read_rtc_mmss();
	ts->tv_nsec = 0;
}

int update_persistent_clock(struct timespec now)
{
	return set_rtc_mmss(now.tv_sec);
}

void time_init(void)
{
	hw_timer_init();
}
