

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/timex.h>
#include <linux/profile.h>

#include <asm/io.h>
#include <asm/timer.h>

#define	TICK_SIZE (tick_nsec / 1000)

void h8300_timer_tick(void)
{
	if (current->pid)
		profile_tick(CPU_PROFILING);
	write_seqlock(&xtime_lock);
	do_timer(1);
	write_sequnlock(&xtime_lock);
	update_process_times(user_mode(get_irq_regs()));
}

void read_persistent_clock(struct timespec *ts)
{
	unsigned int year, mon, day, hour, min, sec;

	/* FIX by dqg : Set to zero for platforms that don't have tod */
	/* without this time is undefined and can overflow time_t, causing  */
	/* very strange errors */
	year = 1980;
	mon = day = 1;
	hour = min = sec = 0;
#ifdef CONFIG_H8300_GETTOD
	h8300_gettod (&year, &mon, &day, &hour, &min, &sec);
#endif
	if ((year += 1900) < 1970)
		year += 100;
	ts->tv_sec = mktime(year, mon, day, hour, min, sec);
	ts->tv_nsec = 0;
}

void __init time_init(void)
{

	h8300_timer_setup();
}
