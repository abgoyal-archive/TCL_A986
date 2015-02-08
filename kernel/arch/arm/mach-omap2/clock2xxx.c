
#undef DEBUG

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <plat/clock.h>

#include "clock.h"
#include "clock2xxx.h"
#include "cm.h"
#include "cm-regbits-24xx.h"

struct clk *vclk, *sclk, *dclk;


void omap2xxx_clk_prepare_for_reboot(void)
{
	u32 rate;

	if (vclk == NULL || sclk == NULL)
		return;

	rate = clk_get_rate(sclk);
	clk_set_rate(vclk, rate);
}

static int __init omap2xxx_clk_arch_init(void)
{
	int ret;

	if (!cpu_is_omap24xx())
		return 0;

	ret = omap2_clk_switch_mpurate_at_boot("virt_prcm_set");
	if (!ret)
		omap2_clk_print_new_rates("sys_ck", "dpll_ck", "mpu_ck");

	return ret;
}

arch_initcall(omap2xxx_clk_arch_init);


