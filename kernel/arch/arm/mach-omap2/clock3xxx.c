
#undef DEBUG

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <plat/clock.h>

#include "clock.h"
#include "clock3xxx.h"
#include "prm.h"
#include "prm-regbits-34xx.h"
#include "cm.h"
#include "cm-regbits-34xx.h"

#define DPLL5_FREQ_FOR_USBHOST		120000000

/* needed by omap3_core_dpll_m2_set_rate() */
struct clk *sdrc_ick_p, *arm_fck_p;

int omap3_dpll4_set_rate(struct clk *clk, unsigned long rate)
{
	/*
	 * According to the 12-5 CDP code from TI, "Limitation 2.5"
	 * on 3430ES1 prevents us from changing DPLL multipliers or dividers
	 * on DPLL4.
	 */
	if (omap_rev() == OMAP3430_REV_ES1_0) {
		pr_err("clock: DPLL4 cannot change rate due to "
		       "silicon 'Limitation 2.5' on 3430ES1.\n");
		return -EINVAL;
	}

	return omap3_noncore_dpll_set_rate(clk, rate);
}

void __init omap3_clk_lock_dpll5(void)
{
	struct clk *dpll5_clk;
	struct clk *dpll5_m2_clk;

	dpll5_clk = clk_get(NULL, "dpll5_ck");
	clk_set_rate(dpll5_clk, DPLL5_FREQ_FOR_USBHOST);
	clk_enable(dpll5_clk);

	/* Enable autoidle to allow it to enter low power bypass */
	omap3_dpll_allow_idle(dpll5_clk);

	/* Program dpll5_m2_clk divider for no division */
	dpll5_m2_clk = clk_get(NULL, "dpll5_m2_ck");
	clk_enable(dpll5_m2_clk);
	clk_set_rate(dpll5_m2_clk, DPLL5_FREQ_FOR_USBHOST);

	clk_disable(dpll5_m2_clk);
	clk_disable(dpll5_clk);
	return;
}

/* Common clock code */

static int __init omap3xxx_clk_arch_init(void)
{
	int ret;

	if (!cpu_is_omap34xx())
		return 0;

	ret = omap2_clk_switch_mpurate_at_boot("dpll1_ck");
	if (!ret)
		omap2_clk_print_new_rates("osc_sys_ck", "arm_fck", "core_ck");

	return ret;
}

arch_initcall(omap3xxx_clk_arch_init);


