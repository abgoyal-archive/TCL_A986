

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/common.h>
#include <asm/proc-fns.h>
#include <asm/system.h>

static void __iomem *wdog_base;

void arch_reset(char mode, const char *cmd)
{
	unsigned int wcr_enable;

#ifdef CONFIG_ARCH_MXC91231
	if (cpu_is_mxc91231()) {
		mxc91231_arch_reset(mode, cmd);
		return;
	}
#endif
	if (cpu_is_mx1()) {
		wcr_enable = (1 << 0);
	} else {
		struct clk *clk;

		clk = clk_get_sys("imx-wdt.0", NULL);
		if (!IS_ERR(clk))
			clk_enable(clk);
		wcr_enable = (1 << 2);
	}

	/* Assert SRS signal */
	__raw_writew(wcr_enable, wdog_base);

	/* wait for reset to assert... */
	mdelay(500);

	printk(KERN_ERR "Watchdog reset failed to assert reset\n");

	/* delay to allow the serial port to show the message */
	mdelay(50);

	/* we'll take a jump through zero as a poor second */
	cpu_reset(0);
}

void mxc_arch_reset_init(void __iomem *base)
{
	wdog_base = base;
}
