
#include <asm/mc146818-time.h>
#include <asm/time.h>

#include <loongson.h>
#include <cs5536/cs5536_mfgpt.h>

void __init plat_time_init(void)
{
	/* setup mips r4k timer */
	mips_hpt_frequency = cpu_clock_freq / 2;

	setup_mfgpt0_timer();
}

void read_persistent_clock(struct timespec *ts)
{
	ts->tv_sec = mc146818_get_cmos_time();
	ts->tv_nsec = 0;
}
