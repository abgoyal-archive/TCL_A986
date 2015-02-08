
#undef DEBUG

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <asm/div64.h>

#include <plat/clock.h>

#include "clock.h"
#include "cm.h"
#include "cm-regbits-24xx.h"
#include "cm-regbits-34xx.h"

/* DPLL rate rounding: minimum DPLL multiplier, divider values */
#define DPLL_MIN_MULTIPLIER		2
#define DPLL_MIN_DIVIDER		1

/* Possible error results from _dpll_test_mult */
#define DPLL_MULT_UNDERFLOW		-1

#define DPLL_SCALE_FACTOR		64
#define DPLL_SCALE_BASE			2
#define DPLL_ROUNDING_VAL		((DPLL_SCALE_BASE / 2) * \
					 (DPLL_SCALE_FACTOR / DPLL_SCALE_BASE))

/* DPLL valid Fint frequency band limits - from 34xx TRM Section 4.7.6.2 */
#define DPLL_FINT_BAND1_MIN		750000
#define DPLL_FINT_BAND1_MAX		2100000
#define DPLL_FINT_BAND2_MIN		7500000
#define DPLL_FINT_BAND2_MAX		21000000

/* _dpll_test_fint() return codes */
#define DPLL_FINT_UNDERFLOW		-1
#define DPLL_FINT_INVALID		-2

/* Private functions */

static int _dpll_test_fint(struct clk *clk, u8 n)
{
	struct dpll_data *dd;
	long fint;
	int ret = 0;

	dd = clk->dpll_data;

	/* DPLL divider must result in a valid jitter correction val */
	fint = clk->parent->rate / (n + 1);
	if (fint < DPLL_FINT_BAND1_MIN) {

		pr_debug("rejecting n=%d due to Fint failure, "
			 "lowering max_divider\n", n);
		dd->max_divider = n;
		ret = DPLL_FINT_UNDERFLOW;

	} else if (fint > DPLL_FINT_BAND1_MAX &&
		   fint < DPLL_FINT_BAND2_MIN) {

		pr_debug("rejecting n=%d due to Fint failure\n", n);
		ret = DPLL_FINT_INVALID;

	} else if (fint > DPLL_FINT_BAND2_MAX) {

		pr_debug("rejecting n=%d due to Fint failure, "
			 "boosting min_divider\n", n);
		dd->min_divider = n;
		ret = DPLL_FINT_INVALID;

	}

	return ret;
}

static unsigned long _dpll_compute_new_rate(unsigned long parent_rate,
					    unsigned int m, unsigned int n)
{
	unsigned long long num;

	num = (unsigned long long)parent_rate * m;
	do_div(num, n);
	return num;
}

static int _dpll_test_mult(int *m, int n, unsigned long *new_rate,
			   unsigned long target_rate,
			   unsigned long parent_rate)
{
	int r = 0, carry = 0;

	/* Unscale m and round if necessary */
	if (*m % DPLL_SCALE_FACTOR >= DPLL_ROUNDING_VAL)
		carry = 1;
	*m = (*m / DPLL_SCALE_FACTOR) + carry;

	/*
	 * The new rate must be <= the target rate to avoid programming
	 * a rate that is impossible for the hardware to handle
	 */
	*new_rate = _dpll_compute_new_rate(parent_rate, *m, n);
	if (*new_rate > target_rate) {
		(*m)--;
		*new_rate = 0;
	}

	/* Guard against m underflow */
	if (*m < DPLL_MIN_MULTIPLIER) {
		*m = DPLL_MIN_MULTIPLIER;
		*new_rate = 0;
		r = DPLL_MULT_UNDERFLOW;
	}

	if (*new_rate == 0)
		*new_rate = _dpll_compute_new_rate(parent_rate, *m, n);

	return r;
}

/* Public functions */

void omap2_init_dpll_parent(struct clk *clk)
{
	u32 v;
	struct dpll_data *dd;

	dd = clk->dpll_data;
	if (!dd)
		return;

	/* Return bypass rate if DPLL is bypassed */
	v = __raw_readl(dd->control_reg);
	v &= dd->enable_mask;
	v >>= __ffs(dd->enable_mask);

	/* Reparent in case the dpll is in bypass */
	if (cpu_is_omap24xx()) {
		if (v == OMAP2XXX_EN_DPLL_LPBYPASS ||
		    v == OMAP2XXX_EN_DPLL_FRBYPASS)
			clk_reparent(clk, dd->clk_bypass);
	} else if (cpu_is_omap34xx()) {
		if (v == OMAP3XXX_EN_DPLL_LPBYPASS ||
		    v == OMAP3XXX_EN_DPLL_FRBYPASS)
			clk_reparent(clk, dd->clk_bypass);
	} else if (cpu_is_omap44xx()) {
		if (v == OMAP4XXX_EN_DPLL_LPBYPASS ||
		    v == OMAP4XXX_EN_DPLL_FRBYPASS ||
		    v == OMAP4XXX_EN_DPLL_MNBYPASS)
			clk_reparent(clk, dd->clk_bypass);
	}
	return;
}

u32 omap2_get_dpll_rate(struct clk *clk)
{
	long long dpll_clk;
	u32 dpll_mult, dpll_div, v;
	struct dpll_data *dd;

	dd = clk->dpll_data;
	if (!dd)
		return 0;

	/* Return bypass rate if DPLL is bypassed */
	v = __raw_readl(dd->control_reg);
	v &= dd->enable_mask;
	v >>= __ffs(dd->enable_mask);

	if (cpu_is_omap24xx()) {
		if (v == OMAP2XXX_EN_DPLL_LPBYPASS ||
		    v == OMAP2XXX_EN_DPLL_FRBYPASS)
			return dd->clk_bypass->rate;
	} else if (cpu_is_omap34xx()) {
		if (v == OMAP3XXX_EN_DPLL_LPBYPASS ||
		    v == OMAP3XXX_EN_DPLL_FRBYPASS)
			return dd->clk_bypass->rate;
	} else if (cpu_is_omap44xx()) {
		if (v == OMAP4XXX_EN_DPLL_LPBYPASS ||
		    v == OMAP4XXX_EN_DPLL_FRBYPASS ||
		    v == OMAP4XXX_EN_DPLL_MNBYPASS)
			return dd->clk_bypass->rate;
	}

	v = __raw_readl(dd->mult_div1_reg);
	dpll_mult = v & dd->mult_mask;
	dpll_mult >>= __ffs(dd->mult_mask);
	dpll_div = v & dd->div1_mask;
	dpll_div >>= __ffs(dd->div1_mask);

	dpll_clk = (long long)dd->clk_ref->rate * dpll_mult;
	do_div(dpll_clk, dpll_div + 1);

	return dpll_clk;
}

/* DPLL rate rounding code */

int omap2_dpll_set_rate_tolerance(struct clk *clk, unsigned int tolerance)
{
	if (!clk || !clk->dpll_data)
		return -EINVAL;

	clk->dpll_data->rate_tolerance = tolerance;

	return 0;
}

long omap2_dpll_round_rate(struct clk *clk, unsigned long target_rate)
{
	int m, n, r, e, scaled_max_m;
	unsigned long scaled_rt_rp, new_rate;
	int min_e = -1, min_e_m = -1, min_e_n = -1;
	struct dpll_data *dd;

	if (!clk || !clk->dpll_data)
		return ~0;

	dd = clk->dpll_data;

	pr_debug("clock: starting DPLL round_rate for clock %s, target rate "
		 "%ld\n", clk->name, target_rate);

	scaled_rt_rp = target_rate / (dd->clk_ref->rate / DPLL_SCALE_FACTOR);
	scaled_max_m = dd->max_multiplier * DPLL_SCALE_FACTOR;

	dd->last_rounded_rate = 0;

	for (n = dd->min_divider; n <= dd->max_divider; n++) {

		/* Is the (input clk, divider) pair valid for the DPLL? */
		r = _dpll_test_fint(clk, n);
		if (r == DPLL_FINT_UNDERFLOW)
			break;
		else if (r == DPLL_FINT_INVALID)
			continue;

		/* Compute the scaled DPLL multiplier, based on the divider */
		m = scaled_rt_rp * n;

		/*
		 * Since we're counting n up, a m overflow means we
		 * can bail out completely (since as n increases in
		 * the next iteration, there's no way that m can
		 * increase beyond the current m)
		 */
		if (m > scaled_max_m)
			break;

		r = _dpll_test_mult(&m, n, &new_rate, target_rate,
				    dd->clk_ref->rate);

		/* m can't be set low enough for this n - try with a larger n */
		if (r == DPLL_MULT_UNDERFLOW)
			continue;

		e = target_rate - new_rate;
		pr_debug("clock: n = %d: m = %d: rate error is %d "
			 "(new_rate = %ld)\n", n, m, e, new_rate);

		if (min_e == -1 ||
		    min_e >= (int)(abs(e) - dd->rate_tolerance)) {
			min_e = e;
			min_e_m = m;
			min_e_n = n;

			pr_debug("clock: found new least error %d\n", min_e);

			/* We found good settings -- bail out now */
			if (min_e <= dd->rate_tolerance)
				break;
		}
	}

	if (min_e < 0) {
		pr_debug("clock: error: target rate or tolerance too low\n");
		return ~0;
	}

	dd->last_rounded_m = min_e_m;
	dd->last_rounded_n = min_e_n;
	dd->last_rounded_rate = _dpll_compute_new_rate(dd->clk_ref->rate,
						       min_e_m,  min_e_n);

	pr_debug("clock: final least error: e = %d, m = %d, n = %d\n",
		 min_e, min_e_m, min_e_n);
	pr_debug("clock: final rate: %ld  (target rate: %ld)\n",
		 dd->last_rounded_rate, target_rate);

	return dd->last_rounded_rate;
}

