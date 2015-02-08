
#undef DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/limits.h>
#include <linux/err.h>

#include <linux/io.h>

#include <linux/bitops.h>

#include "prm.h"
#include "prm-regbits-24xx.h"
#include "cm.h"

#include <plat/clock.h>
#include <plat/powerdomain.h>
#include <plat/clockdomain.h>
#include <plat/prcm.h>

/* clkdm_list contains all registered struct clockdomains */
static LIST_HEAD(clkdm_list);

/* array of clockdomain deps to be added/removed when clkdm in hwsup mode */
static struct clkdm_autodep *autodeps;


/* Private functions */

static struct clockdomain *_clkdm_lookup(const char *name)
{
	struct clockdomain *clkdm, *temp_clkdm;

	if (!name)
		return NULL;

	clkdm = NULL;

	list_for_each_entry(temp_clkdm, &clkdm_list, node) {
		if (!strcmp(name, temp_clkdm->name)) {
			clkdm = temp_clkdm;
			break;
		}
	}

	return clkdm;
}

static int _clkdm_register(struct clockdomain *clkdm)
{
	struct powerdomain *pwrdm;

	if (!clkdm || !clkdm->name)
		return -EINVAL;

	if (!omap_chip_is(clkdm->omap_chip))
		return -EINVAL;

	pwrdm = pwrdm_lookup(clkdm->pwrdm.name);
	if (!pwrdm) {
		pr_err("clockdomain: %s: powerdomain %s does not exist\n",
			clkdm->name, clkdm->pwrdm.name);
		return -EINVAL;
	}
	clkdm->pwrdm.ptr = pwrdm;

	/* Verify that the clockdomain is not already registered */
	if (_clkdm_lookup(clkdm->name))
		return -EEXIST;

	list_add(&clkdm->node, &clkdm_list);

	pwrdm_add_clkdm(pwrdm, clkdm);

	pr_debug("clockdomain: registered %s\n", clkdm->name);

	return 0;
}

/* _clkdm_deps_lookup - look up the specified clockdomain in a clkdm list */
static struct clkdm_dep *_clkdm_deps_lookup(struct clockdomain *clkdm,
					    struct clkdm_dep *deps)
{
	struct clkdm_dep *cd;

	if (!clkdm || !deps || !omap_chip_is(clkdm->omap_chip))
		return ERR_PTR(-EINVAL);

	for (cd = deps; cd->clkdm_name; cd++) {
		if (!omap_chip_is(cd->omap_chip))
			continue;

		if (!cd->clkdm && cd->clkdm_name)
			cd->clkdm = _clkdm_lookup(cd->clkdm_name);

		if (cd->clkdm == clkdm)
			break;
	}

	if (!cd->clkdm_name)
		return ERR_PTR(-ENOENT);

	return cd;
}

static void _autodep_lookup(struct clkdm_autodep *autodep)
{
	struct clockdomain *clkdm;

	if (!autodep)
		return;

	if (!omap_chip_is(autodep->omap_chip))
		return;

	clkdm = clkdm_lookup(autodep->clkdm.name);
	if (!clkdm) {
		pr_err("clockdomain: autodeps: clockdomain %s does not exist\n",
			 autodep->clkdm.name);
		clkdm = ERR_PTR(-ENOENT);
	}
	autodep->clkdm.ptr = clkdm;
}

static void _clkdm_add_autodeps(struct clockdomain *clkdm)
{
	struct clkdm_autodep *autodep;

	if (!autodeps)
		return;

	for (autodep = autodeps; autodep->clkdm.ptr; autodep++) {
		if (IS_ERR(autodep->clkdm.ptr))
			continue;

		if (!omap_chip_is(autodep->omap_chip))
			continue;

		pr_debug("clockdomain: adding %s sleepdep/wkdep for "
			 "clkdm %s\n", autodep->clkdm.ptr->name,
			 clkdm->name);

		clkdm_add_sleepdep(clkdm, autodep->clkdm.ptr);
		clkdm_add_wkdep(clkdm, autodep->clkdm.ptr);
	}
}

static void _clkdm_del_autodeps(struct clockdomain *clkdm)
{
	struct clkdm_autodep *autodep;

	if (!autodeps)
		return;

	for (autodep = autodeps; autodep->clkdm.ptr; autodep++) {
		if (IS_ERR(autodep->clkdm.ptr))
			continue;

		if (!omap_chip_is(autodep->omap_chip))
			continue;

		pr_debug("clockdomain: removing %s sleepdep/wkdep for "
			 "clkdm %s\n", autodep->clkdm.ptr->name,
			 clkdm->name);

		clkdm_del_sleepdep(clkdm, autodep->clkdm.ptr);
		clkdm_del_wkdep(clkdm, autodep->clkdm.ptr);
	}
}

static void _omap2_clkdm_set_hwsup(struct clockdomain *clkdm, int enable)
{
	u32 bits, v;

	if (cpu_is_omap24xx()) {
		if (enable)
			bits = OMAP24XX_CLKSTCTRL_ENABLE_AUTO;
		else
			bits = OMAP24XX_CLKSTCTRL_DISABLE_AUTO;
	} else if (cpu_is_omap34xx() || cpu_is_omap44xx()) {
		if (enable)
			bits = OMAP34XX_CLKSTCTRL_ENABLE_AUTO;
		else
			bits = OMAP34XX_CLKSTCTRL_DISABLE_AUTO;
	} else {
		BUG();
	}

	bits = bits << __ffs(clkdm->clktrctrl_mask);

	v = __raw_readl(clkdm->clkstctrl_reg);
	v &= ~(clkdm->clktrctrl_mask);
	v |= bits;
	__raw_writel(v, clkdm->clkstctrl_reg);

}

static void _init_wkdep_usecount(struct clockdomain *clkdm)
{
	u32 v;
	struct clkdm_dep *cd;

	if (!clkdm->wkdep_srcs)
		return;

	for (cd = clkdm->wkdep_srcs; cd->clkdm_name; cd++) {
		if (!omap_chip_is(cd->omap_chip))
			continue;

		if (!cd->clkdm && cd->clkdm_name)
			cd->clkdm = _clkdm_lookup(cd->clkdm_name);

		if (!cd->clkdm) {
			WARN(!cd->clkdm, "clockdomain: %s: wkdep clkdm %s not "
			     "found\n", clkdm->name, cd->clkdm_name);
			continue;
		}

		v = prm_read_mod_bits_shift(clkdm->pwrdm.ptr->prcm_offs,
					    PM_WKDEP,
					    (1 << cd->clkdm->dep_bit));

		if (v)
			pr_debug("clockdomain: %s: wakeup dependency already "
				 "set to wake up when %s wakes\n",
				 clkdm->name, cd->clkdm->name);

		atomic_set(&cd->wkdep_usecount, (v) ? 1 : 0);
	}
}

static void _init_sleepdep_usecount(struct clockdomain *clkdm)
{
	u32 v;
	struct clkdm_dep *cd;

	if (!cpu_is_omap34xx())
		return;

	if (!clkdm->sleepdep_srcs)
		return;

	for (cd = clkdm->sleepdep_srcs; cd->clkdm_name; cd++) {
		if (!omap_chip_is(cd->omap_chip))
			continue;

		if (!cd->clkdm && cd->clkdm_name)
			cd->clkdm = _clkdm_lookup(cd->clkdm_name);

		if (!cd->clkdm) {
			WARN(!cd->clkdm, "clockdomain: %s: sleepdep clkdm %s "
			     "not found\n", clkdm->name, cd->clkdm_name);
			continue;
		}

		v = prm_read_mod_bits_shift(clkdm->pwrdm.ptr->prcm_offs,
					    OMAP3430_CM_SLEEPDEP,
					    (1 << cd->clkdm->dep_bit));

		if (v)
			pr_debug("clockdomain: %s: sleep dependency already "
				 "set to prevent from idling until %s "
				 "idles\n", clkdm->name, cd->clkdm->name);

		atomic_set(&cd->sleepdep_usecount, (v) ? 1 : 0);
	}
};

/* Public functions */

void clkdm_init(struct clockdomain **clkdms,
		struct clkdm_autodep *init_autodeps)
{
	struct clockdomain **c = NULL;
	struct clockdomain *clkdm;
	struct clkdm_autodep *autodep = NULL;

	if (clkdms)
		for (c = clkdms; *c; c++)
			_clkdm_register(*c);

	autodeps = init_autodeps;
	if (autodeps)
		for (autodep = autodeps; autodep->clkdm.ptr; autodep++)
			_autodep_lookup(autodep);

	/*
	 * Ensure that the *dep_usecount registers reflect the current
	 * state of the PRCM.
	 */
	list_for_each_entry(clkdm, &clkdm_list, node) {
		_init_wkdep_usecount(clkdm);
		_init_sleepdep_usecount(clkdm);
	}
}

struct clockdomain *clkdm_lookup(const char *name)
{
	struct clockdomain *clkdm, *temp_clkdm;

	if (!name)
		return NULL;

	clkdm = NULL;

	list_for_each_entry(temp_clkdm, &clkdm_list, node) {
		if (!strcmp(name, temp_clkdm->name)) {
			clkdm = temp_clkdm;
			break;
		}
	}

	return clkdm;
}

int clkdm_for_each(int (*fn)(struct clockdomain *clkdm, void *user),
			void *user)
{
	struct clockdomain *clkdm;
	int ret = 0;

	if (!fn)
		return -EINVAL;

	list_for_each_entry(clkdm, &clkdm_list, node) {
		ret = (*fn)(clkdm, user);
		if (ret)
			break;
	}

	return ret;
}


struct powerdomain *clkdm_get_pwrdm(struct clockdomain *clkdm)
{
	if (!clkdm)
		return NULL;

	return clkdm->pwrdm.ptr;
}


/* Hardware clockdomain control */

int clkdm_add_wkdep(struct clockdomain *clkdm1, struct clockdomain *clkdm2)
{
	struct clkdm_dep *cd;

	if (!clkdm1 || !clkdm2)
		return -EINVAL;

	cd = _clkdm_deps_lookup(clkdm2, clkdm1->wkdep_srcs);
	if (IS_ERR(cd)) {
		pr_debug("clockdomain: hardware cannot set/clear wake up of "
			 "%s when %s wakes up\n", clkdm1->name, clkdm2->name);
		return PTR_ERR(cd);
	}

	if (atomic_inc_return(&cd->wkdep_usecount) == 1) {
		pr_debug("clockdomain: hardware will wake up %s when %s wakes "
			 "up\n", clkdm1->name, clkdm2->name);

		prm_set_mod_reg_bits((1 << clkdm2->dep_bit),
				     clkdm1->pwrdm.ptr->prcm_offs, PM_WKDEP);
	}

	return 0;
}

int clkdm_del_wkdep(struct clockdomain *clkdm1, struct clockdomain *clkdm2)
{
	struct clkdm_dep *cd;

	if (!clkdm1 || !clkdm2)
		return -EINVAL;

	cd = _clkdm_deps_lookup(clkdm2, clkdm1->wkdep_srcs);
	if (IS_ERR(cd)) {
		pr_debug("clockdomain: hardware cannot set/clear wake up of "
			 "%s when %s wakes up\n", clkdm1->name, clkdm2->name);
		return PTR_ERR(cd);
	}

	if (atomic_dec_return(&cd->wkdep_usecount) == 0) {
		pr_debug("clockdomain: hardware will no longer wake up %s "
			 "after %s wakes up\n", clkdm1->name, clkdm2->name);

		prm_clear_mod_reg_bits((1 << clkdm2->dep_bit),
				       clkdm1->pwrdm.ptr->prcm_offs, PM_WKDEP);
	}

	return 0;
}

int clkdm_read_wkdep(struct clockdomain *clkdm1, struct clockdomain *clkdm2)
{
	struct clkdm_dep *cd;

	if (!clkdm1 || !clkdm2)
		return -EINVAL;

	cd = _clkdm_deps_lookup(clkdm2, clkdm1->wkdep_srcs);
	if (IS_ERR(cd)) {
		pr_debug("clockdomain: hardware cannot set/clear wake up of "
			 "%s when %s wakes up\n", clkdm1->name, clkdm2->name);
		return PTR_ERR(cd);
	}

	/* XXX It's faster to return the atomic wkdep_usecount */
	return prm_read_mod_bits_shift(clkdm1->pwrdm.ptr->prcm_offs, PM_WKDEP,
				       (1 << clkdm2->dep_bit));
}

int clkdm_clear_all_wkdeps(struct clockdomain *clkdm)
{
	struct clkdm_dep *cd;
	u32 mask = 0;

	if (!clkdm)
		return -EINVAL;

	for (cd = clkdm->wkdep_srcs; cd && cd->clkdm_name; cd++) {
		if (!omap_chip_is(cd->omap_chip))
			continue;

		/* PRM accesses are slow, so minimize them */
		mask |= 1 << cd->clkdm->dep_bit;
		atomic_set(&cd->wkdep_usecount, 0);
	}

	prm_clear_mod_reg_bits(mask, clkdm->pwrdm.ptr->prcm_offs, PM_WKDEP);

	return 0;
}

int clkdm_add_sleepdep(struct clockdomain *clkdm1, struct clockdomain *clkdm2)
{
	struct clkdm_dep *cd;

	if (!cpu_is_omap34xx())
		return -EINVAL;

	if (!clkdm1 || !clkdm2)
		return -EINVAL;

	cd = _clkdm_deps_lookup(clkdm2, clkdm1->sleepdep_srcs);
	if (IS_ERR(cd)) {
		pr_debug("clockdomain: hardware cannot set/clear sleep "
			 "dependency affecting %s from %s\n", clkdm1->name,
			 clkdm2->name);
		return PTR_ERR(cd);
	}

	if (atomic_inc_return(&cd->sleepdep_usecount) == 1) {
		pr_debug("clockdomain: will prevent %s from sleeping if %s "
			 "is active\n", clkdm1->name, clkdm2->name);

		cm_set_mod_reg_bits((1 << clkdm2->dep_bit),
				    clkdm1->pwrdm.ptr->prcm_offs,
				    OMAP3430_CM_SLEEPDEP);
	}

	return 0;
}

int clkdm_del_sleepdep(struct clockdomain *clkdm1, struct clockdomain *clkdm2)
{
	struct clkdm_dep *cd;

	if (!cpu_is_omap34xx())
		return -EINVAL;

	if (!clkdm1 || !clkdm2)
		return -EINVAL;

	cd = _clkdm_deps_lookup(clkdm2, clkdm1->sleepdep_srcs);
	if (IS_ERR(cd)) {
		pr_debug("clockdomain: hardware cannot set/clear sleep "
			 "dependency affecting %s from %s\n", clkdm1->name,
			 clkdm2->name);
		return PTR_ERR(cd);
	}

	if (atomic_dec_return(&cd->sleepdep_usecount) == 0) {
		pr_debug("clockdomain: will no longer prevent %s from "
			 "sleeping if %s is active\n", clkdm1->name,
			 clkdm2->name);

		cm_clear_mod_reg_bits((1 << clkdm2->dep_bit),
				      clkdm1->pwrdm.ptr->prcm_offs,
				      OMAP3430_CM_SLEEPDEP);
	}

	return 0;
}

int clkdm_read_sleepdep(struct clockdomain *clkdm1, struct clockdomain *clkdm2)
{
	struct clkdm_dep *cd;

	if (!cpu_is_omap34xx())
		return -EINVAL;

	if (!clkdm1 || !clkdm2)
		return -EINVAL;

	cd = _clkdm_deps_lookup(clkdm2, clkdm1->sleepdep_srcs);
	if (IS_ERR(cd)) {
		pr_debug("clockdomain: hardware cannot set/clear sleep "
			 "dependency affecting %s from %s\n", clkdm1->name,
			 clkdm2->name);
		return PTR_ERR(cd);
	}

	/* XXX It's faster to return the atomic sleepdep_usecount */
	return prm_read_mod_bits_shift(clkdm1->pwrdm.ptr->prcm_offs,
				       OMAP3430_CM_SLEEPDEP,
				       (1 << clkdm2->dep_bit));
}

int clkdm_clear_all_sleepdeps(struct clockdomain *clkdm)
{
	struct clkdm_dep *cd;
	u32 mask = 0;

	if (!cpu_is_omap34xx())
		return -EINVAL;

	if (!clkdm)
		return -EINVAL;

	for (cd = clkdm->sleepdep_srcs; cd && cd->clkdm_name; cd++) {
		if (!omap_chip_is(cd->omap_chip))
			continue;

		/* PRM accesses are slow, so minimize them */
		mask |= 1 << cd->clkdm->dep_bit;
		atomic_set(&cd->sleepdep_usecount, 0);
	}

	prm_clear_mod_reg_bits(mask, clkdm->pwrdm.ptr->prcm_offs,
			       OMAP3430_CM_SLEEPDEP);

	return 0;
}

static int omap2_clkdm_clktrctrl_read(struct clockdomain *clkdm)
{
	u32 v;

	if (!clkdm)
		return -EINVAL;

	v = __raw_readl(clkdm->clkstctrl_reg);
	v &= clkdm->clktrctrl_mask;
	v >>= __ffs(clkdm->clktrctrl_mask);

	return v;
}

int omap2_clkdm_sleep(struct clockdomain *clkdm)
{
	if (!clkdm)
		return -EINVAL;

	if (!(clkdm->flags & CLKDM_CAN_FORCE_SLEEP)) {
		pr_debug("clockdomain: %s does not support forcing "
			 "sleep via software\n", clkdm->name);
		return -EINVAL;
	}

	pr_debug("clockdomain: forcing sleep on %s\n", clkdm->name);

	if (cpu_is_omap24xx()) {

		cm_set_mod_reg_bits(OMAP24XX_FORCESTATE_MASK,
			    clkdm->pwrdm.ptr->prcm_offs, OMAP2_PM_PWSTCTRL);

	} else if (cpu_is_omap34xx() || cpu_is_omap44xx()) {

		u32 bits = (OMAP34XX_CLKSTCTRL_FORCE_SLEEP <<
			 __ffs(clkdm->clktrctrl_mask));

		u32 v = __raw_readl(clkdm->clkstctrl_reg);
		v &= ~(clkdm->clktrctrl_mask);
		v |= bits;
		__raw_writel(v, clkdm->clkstctrl_reg);

	} else {
		BUG();
	};

	return 0;
}

int omap2_clkdm_wakeup(struct clockdomain *clkdm)
{
	if (!clkdm)
		return -EINVAL;

	if (!(clkdm->flags & CLKDM_CAN_FORCE_WAKEUP)) {
		pr_debug("clockdomain: %s does not support forcing "
			 "wakeup via software\n", clkdm->name);
		return -EINVAL;
	}

	pr_debug("clockdomain: forcing wakeup on %s\n", clkdm->name);

	if (cpu_is_omap24xx()) {

		cm_clear_mod_reg_bits(OMAP24XX_FORCESTATE_MASK,
			      clkdm->pwrdm.ptr->prcm_offs, OMAP2_PM_PWSTCTRL);

	} else if (cpu_is_omap34xx() || cpu_is_omap44xx()) {

		u32 bits = (OMAP34XX_CLKSTCTRL_FORCE_WAKEUP <<
			 __ffs(clkdm->clktrctrl_mask));

		u32 v = __raw_readl(clkdm->clkstctrl_reg);
		v &= ~(clkdm->clktrctrl_mask);
		v |= bits;
		__raw_writel(v, clkdm->clkstctrl_reg);

	} else {
		BUG();
	};

	return 0;
}

void omap2_clkdm_allow_idle(struct clockdomain *clkdm)
{
	if (!clkdm)
		return;

	if (!(clkdm->flags & CLKDM_CAN_ENABLE_AUTO)) {
		pr_debug("clock: automatic idle transitions cannot be enabled "
			 "on clockdomain %s\n", clkdm->name);
		return;
	}

	pr_debug("clockdomain: enabling automatic idle transitions for %s\n",
		 clkdm->name);

	/*
	 * XXX This should be removed once TI adds wakeup/sleep
	 * dependency code and data for OMAP4.
	 */
	if (cpu_is_omap44xx()) {
		WARN_ONCE(1, "clockdomain: OMAP4 wakeup/sleep dependency "
			  "support is not yet implemented\n");
	} else {
		if (atomic_read(&clkdm->usecount) > 0)
			_clkdm_add_autodeps(clkdm);
	}

	_omap2_clkdm_set_hwsup(clkdm, 1);

	pwrdm_clkdm_state_switch(clkdm);
}

void omap2_clkdm_deny_idle(struct clockdomain *clkdm)
{
	if (!clkdm)
		return;

	if (!(clkdm->flags & CLKDM_CAN_DISABLE_AUTO)) {
		pr_debug("clockdomain: automatic idle transitions cannot be "
			 "disabled on %s\n", clkdm->name);
		return;
	}

	pr_debug("clockdomain: disabling automatic idle transitions for %s\n",
		 clkdm->name);

	_omap2_clkdm_set_hwsup(clkdm, 0);

	/*
	 * XXX This should be removed once TI adds wakeup/sleep
	 * dependency code and data for OMAP4.
	 */
	if (cpu_is_omap44xx()) {
		WARN_ONCE(1, "clockdomain: OMAP4 wakeup/sleep dependency "
			  "support is not yet implemented\n");
	} else {
		if (atomic_read(&clkdm->usecount) > 0)
			_clkdm_del_autodeps(clkdm);
	}
}


/* Clockdomain-to-clock framework interface code */

int omap2_clkdm_clk_enable(struct clockdomain *clkdm, struct clk *clk)
{
	int v;

	/*
	 * XXX Rewrite this code to maintain a list of enabled
	 * downstream clocks for debugging purposes?
	 */

	if (!clkdm || !clk)
		return -EINVAL;

	if (atomic_inc_return(&clkdm->usecount) > 1)
		return 0;

	/* Clockdomain now has one enabled downstream clock */

	pr_debug("clockdomain: clkdm %s: clk %s now enabled\n", clkdm->name,
		 clk->name);

	if (!clkdm->clkstctrl_reg)
		return 0;

	v = omap2_clkdm_clktrctrl_read(clkdm);

	if ((cpu_is_omap34xx() && v == OMAP34XX_CLKSTCTRL_ENABLE_AUTO) ||
	    (cpu_is_omap24xx() && v == OMAP24XX_CLKSTCTRL_ENABLE_AUTO)) {
		/* Disable HW transitions when we are changing deps */
		_omap2_clkdm_set_hwsup(clkdm, 0);
		_clkdm_add_autodeps(clkdm);
		_omap2_clkdm_set_hwsup(clkdm, 1);
	} else {
		omap2_clkdm_wakeup(clkdm);
	}

	pwrdm_wait_transition(clkdm->pwrdm.ptr);
	pwrdm_clkdm_state_switch(clkdm);

	return 0;
}

int omap2_clkdm_clk_disable(struct clockdomain *clkdm, struct clk *clk)
{
	int v;

	/*
	 * XXX Rewrite this code to maintain a list of enabled
	 * downstream clocks for debugging purposes?
	 */

	if (!clkdm || !clk)
		return -EINVAL;

#ifdef DEBUG
	if (atomic_read(&clkdm->usecount) == 0) {
		WARN_ON(1); /* underflow */
		return -ERANGE;
	}
#endif

	if (atomic_dec_return(&clkdm->usecount) > 0)
		return 0;

	/* All downstream clocks of this clockdomain are now disabled */

	pr_debug("clockdomain: clkdm %s: clk %s now disabled\n", clkdm->name,
		 clk->name);

	if (!clkdm->clkstctrl_reg)
		return 0;

	v = omap2_clkdm_clktrctrl_read(clkdm);

	if ((cpu_is_omap34xx() && v == OMAP34XX_CLKSTCTRL_ENABLE_AUTO) ||
	    (cpu_is_omap24xx() && v == OMAP24XX_CLKSTCTRL_ENABLE_AUTO)) {
		/* Disable HW transitions when we are changing deps */
		_omap2_clkdm_set_hwsup(clkdm, 0);
		_clkdm_del_autodeps(clkdm);
		_omap2_clkdm_set_hwsup(clkdm, 1);
	} else {
		omap2_clkdm_sleep(clkdm);
	}

	pwrdm_clkdm_state_switch(clkdm);

	return 0;
}

