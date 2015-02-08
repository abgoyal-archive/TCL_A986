

#include <linux/init.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/sysctl.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <asm/uaccess.h>

#include <asm/mb86943a.h>

#include "local.h"

static unsigned long imask;
static void mb93093_power_switch_setup(void)
{
	/* mask all but FPGA interrupt sources. */
	imask = *(volatile unsigned long *)0xfeff9820;
	*(volatile unsigned long *)0xfeff9820 = ~(1 << (IRQ_XIRQ2_LEVEL + 16)) & 0xfffe0000;
}

static void mb93093_power_switch_cleanup(void)
{
	*(volatile unsigned long *)0xfeff9820 = imask;
}

static int mb93093_power_switch_check(void)
{
	return 1;
}

static int __init mb93093_pm_init(void)
{
	__power_switch_wake_setup = mb93093_power_switch_setup;
	__power_switch_wake_check = mb93093_power_switch_check;
	__power_switch_wake_cleanup = mb93093_power_switch_cleanup;
	return 0;
}

__initcall(mb93093_pm_init);

