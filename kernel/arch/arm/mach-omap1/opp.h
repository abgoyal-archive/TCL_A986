

#ifndef __ARCH_ARM_MACH_OMAP1_OPP_H
#define __ARCH_ARM_MACH_OMAP1_OPP_H

#include <linux/types.h>

struct mpu_rate {
	unsigned long		rate;
	unsigned long		xtal;
	unsigned long		pll_rate;
	__u16			ckctl_val;
	__u16			dpllctl_val;
};

extern struct mpu_rate omap1_rate_table[];

#endif
