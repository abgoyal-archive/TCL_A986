
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <mach/common.h>
#include <asm/mach/map.h>

void __init shmobile_setup_console(void)
{
	parse_early_param();

	/* Let earlyprintk output early console messages */
	early_platform_driver_probe("earlyprintk", 1, 1);
}
