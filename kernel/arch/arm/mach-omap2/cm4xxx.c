

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/io.h>

#include <asm/atomic.h>

#include <plat/common.h>

#include "cm.h"
#include "cm-regbits-44xx.h"

int omap4_cm_wait_module_ready(void __iomem *clkctrl_reg)
{
	int i = 0;

	if (!clkctrl_reg)
		return 0;

	omap_test_timeout(((__raw_readl(clkctrl_reg) &
			    OMAP4430_IDLEST_MASK) == 0),
			  MAX_MODULE_READY_TIME, i);

	return (i < MAX_MODULE_READY_TIME) ? 0 : -EBUSY;
}

