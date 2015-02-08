


#include <linux/init.h>
#include <linux/ssb/ssb.h>
#include <asm/time.h>
#include <bcm47xx.h>

void __init plat_time_init(void)
{
	unsigned long hz;

	/*
	 * Use deterministic values for initial counter interrupt
	 * so that calibrate delay avoids encountering a counter wrap.
	 */
	write_c0_count(0);
	write_c0_compare(0xffff);

	hz = ssb_cpu_clock(&ssb_bcm47xx.mipscore) / 2;
	if (!hz)
		hz = 100000000;

	/* Set MIPS counter frequency for fixed_rate_gettimeoffset() */
	mips_hpt_frequency = hz;
}
