
#include <linux/platform_device.h>
#include <asm/mach/time.h>

static void __init shmobile_late_time_init(void)
{
	/*
	 * Make sure all compiled-in early timers register themselves.
	 *
	 * Run probe() for two "earlytimer" devices, these will be the
	 * clockevents and clocksource devices respectively. In the event
	 * that only a clockevents device is available, we -ENODEV on the
	 * clocksource and the jiffies clocksource is used transparently
	 * instead. No error handling is necessary here.
	 */
	early_platform_driver_register_all("earlytimer");
	early_platform_driver_probe("earlytimer", 2, 0);
}

static void __init shmobile_timer_init(void)
{
	late_time_init = shmobile_late_time_init;
}

struct sys_timer shmobile_timer = {
	.init		= shmobile_timer_init,
};
