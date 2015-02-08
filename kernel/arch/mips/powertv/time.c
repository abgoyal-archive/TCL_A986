

#include <linux/init.h>
#include <asm/mach-powertv/interrupts.h>
#include <asm/time.h>

#include "powertv-clock.h"

unsigned int __cpuinit get_c0_compare_int(void)
{
	return irq_mips_timer;
}

void __init plat_time_init(void)
{
	powertv_clocksource_init();
}
