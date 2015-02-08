

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/sysdev.h>
#include <linux/device.h>
#include <linux/io.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/hardware.h>

#include <mach/map.h>
#include <mach/regs-gpio.h>
#include <mach/regs-mem.h>

#include <asm/mach-types.h>

#include <plat/pm.h>

#define COPYRIGHT ", Copyright 2005 Simtec Electronics"


static __init int pm_simtec_init(void)
{
	unsigned long gstatus4;

	/* check which machine we are running on */

	if (!machine_is_bast() && !machine_is_vr1000() &&
	    !machine_is_anubis() && !machine_is_osiris() &&
	    !machine_is_aml_m5900())
		return 0;

	printk(KERN_INFO "Simtec Board Power Manangement" COPYRIGHT "\n");

	gstatus4  = (__raw_readl(S3C2410_BANKCON7) & 0x3) << 30;
	gstatus4 |= (__raw_readl(S3C2410_BANKCON6) & 0x3) << 28;
	gstatus4 |= (__raw_readl(S3C2410_BANKSIZE) & S3C2410_BANKSIZE_MASK);

	__raw_writel(gstatus4, S3C2410_GSTATUS4);

	return s3c_pm_init();
}

arch_initcall(pm_simtec_init);
