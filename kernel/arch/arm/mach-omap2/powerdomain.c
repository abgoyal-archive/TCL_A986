
#undef DEBUG

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

#include "cm.h"
#include "cm-regbits-34xx.h"
#include "cm-regbits-44xx.h"
#include "prm.h"
#include "prm-regbits-34xx.h"
#include "prm-regbits-44xx.h"

#include <plat/cpu.h>
#include <plat/powerdomain.h>
#include <plat/clockdomain.h>
#include <plat/prcm.h>

#include "pm.h"

enum {
	PWRDM_STATE_NOW = 0,
	PWRDM_STATE_PREV,
};

/* Variable holding value of the CPU dependent PWRSTCTRL Register Offset */
static u16 pwrstctrl_reg_offs;

/* Variable holding value of the CPU dependent PWRSTST Register Offset */
static u16 pwrstst_reg_offs;


/* OMAP3 and OMAP4 Memory Onstate Masks (common across all power domains) */
#define OMAP_MEM0_ONSTATE_MASK OMAP3430_SHAREDL1CACHEFLATONSTATE_MASK
#define OMAP_MEM1_ONSTATE_MASK OMAP3430_L1FLATMEMONSTATE_MASK
#define OMAP_MEM2_ONSTATE_MASK OMAP3430_SHAREDL2CACHEFLATONSTATE_MASK
#define OMAP_MEM3_ONSTATE_MASK OMAP3430_L2FLATMEMONSTATE_MASK
#define OMAP_MEM4_ONSTATE_MASK OMAP4430_OCP_NRET_BANK_ONSTATE_MASK

/* OMAP3 and OMAP4 Memory Retstate Masks (common across all power domains) */
#define OMAP_MEM0_RETSTATE_MASK OMAP3430_SHAREDL1CACHEFLATRETSTATE_MASK
#define OMAP_MEM1_RETSTATE_MASK OMAP3430_L1FLATMEMRETSTATE_MASK
#define OMAP_MEM2_RETSTATE_MASK OMAP3430_SHAREDL2CACHEFLATRETSTATE_MASK
#define OMAP_MEM3_RETSTATE_MASK OMAP3430_L2FLATMEMRETSTATE_MASK
#define OMAP_MEM4_RETSTATE_MASK OMAP4430_OCP_NRET_BANK_RETSTATE_MASK

/* OMAP3 and OMAP4 Memory Status bits */
#define OMAP_MEM0_STATEST_MASK OMAP3430_SHAREDL1CACHEFLATSTATEST_MASK
#define OMAP_MEM1_STATEST_MASK OMAP3430_L1FLATMEMSTATEST_MASK
#define OMAP_MEM2_STATEST_MASK OMAP3430_SHAREDL2CACHEFLATSTATEST_MASK
#define OMAP_MEM3_STATEST_MASK OMAP3430_L2FLATMEMSTATEST_MASK
#define OMAP_MEM4_STATEST_MASK OMAP4430_OCP_NRET_BANK_STATEST_MASK

/* pwrdm_list contains all registered struct powerdomains */
static LIST_HEAD(pwrdm_list);

/* Private functions */

static struct powerdomain *_pwrdm_lookup(const char *name)
{
	struct powerdomain *pwrdm, *temp_pwrdm;

	pwrdm = NULL;

	list_for_each_entry(temp_pwrdm, &pwrdm_list, node) {
		if (!strcmp(name, temp_pwrdm->name)) {
			pwrdm = temp_pwrdm;
			break;
		}
	}

	return pwrdm;
}

static int _pwrdm_register(struct powerdomain *pwrdm)
{
	int i;

	if (!pwrdm)
		return -EINVAL;

	if (!omap_chip_is(pwrdm->omap_chip))
		return -EINVAL;

	if (_pwrdm_lookup(pwrdm->name))
		return -EEXIST;

	list_add(&pwrdm->node, &pwrdm_list);

	/* Initialize the powerdomain's state counter */
	for (i = 0; i < PWRDM_MAX_PWRSTS; i++)
		pwrdm->state_counter[i] = 0;

	pwrdm->ret_logic_off_counter = 0;
	for (i = 0; i < pwrdm->banks; i++)
		pwrdm->ret_mem_off_counter[i] = 0;

	pwrdm_wait_transition(pwrdm);
	pwrdm->state = pwrdm_read_pwrst(pwrdm);
	pwrdm->state_counter[pwrdm->state] = 1;

	pr_debug("powerdomain: registered %s\n", pwrdm->name);

	return 0;
}

static void _update_logic_membank_counters(struct powerdomain *pwrdm)
{
	int i;
	u8 prev_logic_pwrst, prev_mem_pwrst;

	prev_logic_pwrst = pwrdm_read_prev_logic_pwrst(pwrdm);
	if ((pwrdm->pwrsts_logic_ret == PWRSTS_OFF_RET) &&
	    (prev_logic_pwrst == PWRDM_POWER_OFF))
		pwrdm->ret_logic_off_counter++;

	for (i = 0; i < pwrdm->banks; i++) {
		prev_mem_pwrst = pwrdm_read_prev_mem_pwrst(pwrdm, i);

		if ((pwrdm->pwrsts_mem_ret[i] == PWRSTS_OFF_RET) &&
		    (prev_mem_pwrst == PWRDM_POWER_OFF))
			pwrdm->ret_mem_off_counter[i]++;
	}
}

static int _pwrdm_state_switch(struct powerdomain *pwrdm, int flag)
{

	int prev;
	int state;

	if (pwrdm == NULL)
		return -EINVAL;

	state = pwrdm_read_pwrst(pwrdm);

	switch (flag) {
	case PWRDM_STATE_NOW:
		prev = pwrdm->state;
		break;
	case PWRDM_STATE_PREV:
		prev = pwrdm_read_prev_pwrst(pwrdm);
		if (pwrdm->state != prev)
			pwrdm->state_counter[prev]++;
		if (prev == PWRDM_POWER_RET)
			_update_logic_membank_counters(pwrdm);
		break;
	default:
		return -EINVAL;
	}

	if (state != prev)
		pwrdm->state_counter[state]++;

	pm_dbg_update_time(pwrdm, prev);

	pwrdm->state = state;

	return 0;
}

static int _pwrdm_pre_transition_cb(struct powerdomain *pwrdm, void *unused)
{
	pwrdm_clear_all_prev_pwrst(pwrdm);
	_pwrdm_state_switch(pwrdm, PWRDM_STATE_NOW);
	return 0;
}

static int _pwrdm_post_transition_cb(struct powerdomain *pwrdm, void *unused)
{
	_pwrdm_state_switch(pwrdm, PWRDM_STATE_PREV);
	return 0;
}

/* Public functions */

void pwrdm_init(struct powerdomain **pwrdm_list)
{
	struct powerdomain **p = NULL;

	if (cpu_is_omap24xx() || cpu_is_omap34xx()) {
		pwrstctrl_reg_offs = OMAP2_PM_PWSTCTRL;
		pwrstst_reg_offs = OMAP2_PM_PWSTST;
	} else if (cpu_is_omap44xx()) {
		pwrstctrl_reg_offs = OMAP4_PM_PWSTCTRL;
		pwrstst_reg_offs = OMAP4_PM_PWSTST;
	} else {
		printk(KERN_ERR "Power Domain struct not supported for " \
							"this CPU\n");
		return;
	}

	if (pwrdm_list) {
		for (p = pwrdm_list; *p; p++)
			_pwrdm_register(*p);
	}
}

struct powerdomain *pwrdm_lookup(const char *name)
{
	struct powerdomain *pwrdm;

	if (!name)
		return NULL;

	pwrdm = _pwrdm_lookup(name);

	return pwrdm;
}

int pwrdm_for_each(int (*fn)(struct powerdomain *pwrdm, void *user),
		   void *user)
{
	struct powerdomain *temp_pwrdm;
	int ret = 0;

	if (!fn)
		return -EINVAL;

	list_for_each_entry(temp_pwrdm, &pwrdm_list, node) {
		ret = (*fn)(temp_pwrdm, user);
		if (ret)
			break;
	}

	return ret;
}

int pwrdm_add_clkdm(struct powerdomain *pwrdm, struct clockdomain *clkdm)
{
	int i;
	int ret = -EINVAL;

	if (!pwrdm || !clkdm)
		return -EINVAL;

	pr_debug("powerdomain: associating clockdomain %s with powerdomain "
		 "%s\n", clkdm->name, pwrdm->name);

	for (i = 0; i < PWRDM_MAX_CLKDMS; i++) {
		if (!pwrdm->pwrdm_clkdms[i])
			break;
#ifdef DEBUG
		if (pwrdm->pwrdm_clkdms[i] == clkdm) {
			ret = -EINVAL;
			goto pac_exit;
		}
#endif
	}

	if (i == PWRDM_MAX_CLKDMS) {
		pr_debug("powerdomain: increase PWRDM_MAX_CLKDMS for "
			 "pwrdm %s clkdm %s\n", pwrdm->name, clkdm->name);
		WARN_ON(1);
		ret = -ENOMEM;
		goto pac_exit;
	}

	pwrdm->pwrdm_clkdms[i] = clkdm;

	ret = 0;

pac_exit:
	return ret;
}

int pwrdm_del_clkdm(struct powerdomain *pwrdm, struct clockdomain *clkdm)
{
	int ret = -EINVAL;
	int i;

	if (!pwrdm || !clkdm)
		return -EINVAL;

	pr_debug("powerdomain: dissociating clockdomain %s from powerdomain "
		 "%s\n", clkdm->name, pwrdm->name);

	for (i = 0; i < PWRDM_MAX_CLKDMS; i++)
		if (pwrdm->pwrdm_clkdms[i] == clkdm)
			break;

	if (i == PWRDM_MAX_CLKDMS) {
		pr_debug("powerdomain: clkdm %s not associated with pwrdm "
			 "%s ?!\n", clkdm->name, pwrdm->name);
		ret = -ENOENT;
		goto pdc_exit;
	}

	pwrdm->pwrdm_clkdms[i] = NULL;

	ret = 0;

pdc_exit:
	return ret;
}

int pwrdm_for_each_clkdm(struct powerdomain *pwrdm,
			 int (*fn)(struct powerdomain *pwrdm,
				   struct clockdomain *clkdm))
{
	int ret = 0;
	int i;

	if (!fn)
		return -EINVAL;

	for (i = 0; i < PWRDM_MAX_CLKDMS && !ret; i++)
		ret = (*fn)(pwrdm, pwrdm->pwrdm_clkdms[i]);

	return ret;
}

int pwrdm_get_mem_bank_count(struct powerdomain *pwrdm)
{
	if (!pwrdm)
		return -EINVAL;

	return pwrdm->banks;
}

int pwrdm_set_next_pwrst(struct powerdomain *pwrdm, u8 pwrst)
{
	if (!pwrdm)
		return -EINVAL;

	if (!(pwrdm->pwrsts & (1 << pwrst)))
		return -EINVAL;

	pr_debug("powerdomain: setting next powerstate for %s to %0x\n",
		 pwrdm->name, pwrst);

	prm_rmw_mod_reg_bits(OMAP_POWERSTATE_MASK,
			     (pwrst << OMAP_POWERSTATE_SHIFT),
			     pwrdm->prcm_offs, pwrstctrl_reg_offs);

	return 0;
}

int pwrdm_read_next_pwrst(struct powerdomain *pwrdm)
{
	if (!pwrdm)
		return -EINVAL;

	return prm_read_mod_bits_shift(pwrdm->prcm_offs,
				 pwrstctrl_reg_offs, OMAP_POWERSTATE_MASK);
}

int pwrdm_read_pwrst(struct powerdomain *pwrdm)
{
	if (!pwrdm)
		return -EINVAL;

	return prm_read_mod_bits_shift(pwrdm->prcm_offs,
				 pwrstst_reg_offs, OMAP_POWERSTATEST_MASK);
}

int pwrdm_read_prev_pwrst(struct powerdomain *pwrdm)
{
	if (!pwrdm)
		return -EINVAL;

	return prm_read_mod_bits_shift(pwrdm->prcm_offs, OMAP3430_PM_PREPWSTST,
					OMAP3430_LASTPOWERSTATEENTERED_MASK);
}

int pwrdm_set_logic_retst(struct powerdomain *pwrdm, u8 pwrst)
{
	u32 v;

	if (!pwrdm)
		return -EINVAL;

	if (!(pwrdm->pwrsts_logic_ret & (1 << pwrst)))
		return -EINVAL;

	pr_debug("powerdomain: setting next logic powerstate for %s to %0x\n",
		 pwrdm->name, pwrst);

	/*
	 * The register bit names below may not correspond to the
	 * actual names of the bits in each powerdomain's register,
	 * but the type of value returned is the same for each
	 * powerdomain.
	 */
	v = pwrst << __ffs(OMAP3430_LOGICL1CACHERETSTATE_MASK);
	prm_rmw_mod_reg_bits(OMAP3430_LOGICL1CACHERETSTATE_MASK, v,
			     pwrdm->prcm_offs, pwrstctrl_reg_offs);

	return 0;
}

int pwrdm_set_mem_onst(struct powerdomain *pwrdm, u8 bank, u8 pwrst)
{
	u32 m;

	if (!pwrdm)
		return -EINVAL;

	if (pwrdm->banks < (bank + 1))
		return -EEXIST;

	if (!(pwrdm->pwrsts_mem_on[bank] & (1 << pwrst)))
		return -EINVAL;

	pr_debug("powerdomain: setting next memory powerstate for domain %s "
		 "bank %0x while pwrdm-ON to %0x\n", pwrdm->name, bank, pwrst);

	/*
	 * The register bit names below may not correspond to the
	 * actual names of the bits in each powerdomain's register,
	 * but the type of value returned is the same for each
	 * powerdomain.
	 */
	switch (bank) {
	case 0:
		m = OMAP_MEM0_ONSTATE_MASK;
		break;
	case 1:
		m = OMAP_MEM1_ONSTATE_MASK;
		break;
	case 2:
		m = OMAP_MEM2_ONSTATE_MASK;
		break;
	case 3:
		m = OMAP_MEM3_ONSTATE_MASK;
		break;
	case 4:
		m = OMAP_MEM4_ONSTATE_MASK;
		break;
	default:
		WARN_ON(1); /* should never happen */
		return -EEXIST;
	}

	prm_rmw_mod_reg_bits(m, (pwrst << __ffs(m)),
			     pwrdm->prcm_offs, pwrstctrl_reg_offs);

	return 0;
}

int pwrdm_set_mem_retst(struct powerdomain *pwrdm, u8 bank, u8 pwrst)
{
	u32 m;

	if (!pwrdm)
		return -EINVAL;

	if (pwrdm->banks < (bank + 1))
		return -EEXIST;

	if (!(pwrdm->pwrsts_mem_ret[bank] & (1 << pwrst)))
		return -EINVAL;

	pr_debug("powerdomain: setting next memory powerstate for domain %s "
		 "bank %0x while pwrdm-RET to %0x\n", pwrdm->name, bank, pwrst);

	/*
	 * The register bit names below may not correspond to the
	 * actual names of the bits in each powerdomain's register,
	 * but the type of value returned is the same for each
	 * powerdomain.
	 */
	switch (bank) {
	case 0:
		m = OMAP_MEM0_RETSTATE_MASK;
		break;
	case 1:
		m = OMAP_MEM1_RETSTATE_MASK;
		break;
	case 2:
		m = OMAP_MEM2_RETSTATE_MASK;
		break;
	case 3:
		m = OMAP_MEM3_RETSTATE_MASK;
		break;
	case 4:
		m = OMAP_MEM4_RETSTATE_MASK;
		break;
	default:
		WARN_ON(1); /* should never happen */
		return -EEXIST;
	}

	prm_rmw_mod_reg_bits(m, (pwrst << __ffs(m)), pwrdm->prcm_offs,
			     pwrstctrl_reg_offs);

	return 0;
}

int pwrdm_read_logic_pwrst(struct powerdomain *pwrdm)
{
	if (!pwrdm)
		return -EINVAL;

	return prm_read_mod_bits_shift(pwrdm->prcm_offs, pwrstst_reg_offs,
				       OMAP3430_LOGICSTATEST_MASK);
}

int pwrdm_read_prev_logic_pwrst(struct powerdomain *pwrdm)
{
	if (!pwrdm)
		return -EINVAL;

	/*
	 * The register bit names below may not correspond to the
	 * actual names of the bits in each powerdomain's register,
	 * but the type of value returned is the same for each
	 * powerdomain.
	 */
	return prm_read_mod_bits_shift(pwrdm->prcm_offs, OMAP3430_PM_PREPWSTST,
					OMAP3430_LASTLOGICSTATEENTERED_MASK);
}

int pwrdm_read_logic_retst(struct powerdomain *pwrdm)
{
	if (!pwrdm)
		return -EINVAL;

	/*
	 * The register bit names below may not correspond to the
	 * actual names of the bits in each powerdomain's register,
	 * but the type of value returned is the same for each
	 * powerdomain.
	 */
	return prm_read_mod_bits_shift(pwrdm->prcm_offs, pwrstctrl_reg_offs,
				       OMAP3430_LOGICSTATEST_MASK);
}

int pwrdm_read_mem_pwrst(struct powerdomain *pwrdm, u8 bank)
{
	u32 m;

	if (!pwrdm)
		return -EINVAL;

	if (pwrdm->banks < (bank + 1))
		return -EEXIST;

	if (pwrdm->flags & PWRDM_HAS_MPU_QUIRK)
		bank = 1;

	/*
	 * The register bit names below may not correspond to the
	 * actual names of the bits in each powerdomain's register,
	 * but the type of value returned is the same for each
	 * powerdomain.
	 */
	switch (bank) {
	case 0:
		m = OMAP_MEM0_STATEST_MASK;
		break;
	case 1:
		m = OMAP_MEM1_STATEST_MASK;
		break;
	case 2:
		m = OMAP_MEM2_STATEST_MASK;
		break;
	case 3:
		m = OMAP_MEM3_STATEST_MASK;
		break;
	case 4:
		m = OMAP_MEM4_STATEST_MASK;
		break;
	default:
		WARN_ON(1); /* should never happen */
		return -EEXIST;
	}

	return prm_read_mod_bits_shift(pwrdm->prcm_offs,
					 pwrstst_reg_offs, m);
}

int pwrdm_read_prev_mem_pwrst(struct powerdomain *pwrdm, u8 bank)
{
	u32 m;

	if (!pwrdm)
		return -EINVAL;

	if (pwrdm->banks < (bank + 1))
		return -EEXIST;

	if (pwrdm->flags & PWRDM_HAS_MPU_QUIRK)
		bank = 1;

	/*
	 * The register bit names below may not correspond to the
	 * actual names of the bits in each powerdomain's register,
	 * but the type of value returned is the same for each
	 * powerdomain.
	 */
	switch (bank) {
	case 0:
		m = OMAP3430_LASTMEM1STATEENTERED_MASK;
		break;
	case 1:
		m = OMAP3430_LASTMEM2STATEENTERED_MASK;
		break;
	case 2:
		m = OMAP3430_LASTSHAREDL2CACHEFLATSTATEENTERED_MASK;
		break;
	case 3:
		m = OMAP3430_LASTL2FLATMEMSTATEENTERED_MASK;
		break;
	default:
		WARN_ON(1); /* should never happen */
		return -EEXIST;
	}

	return prm_read_mod_bits_shift(pwrdm->prcm_offs,
					OMAP3430_PM_PREPWSTST, m);
}

int pwrdm_read_mem_retst(struct powerdomain *pwrdm, u8 bank)
{
	u32 m;

	if (!pwrdm)
		return -EINVAL;

	if (pwrdm->banks < (bank + 1))
		return -EEXIST;

	/*
	 * The register bit names below may not correspond to the
	 * actual names of the bits in each powerdomain's register,
	 * but the type of value returned is the same for each
	 * powerdomain.
	 */
	switch (bank) {
	case 0:
		m = OMAP_MEM0_RETSTATE_MASK;
		break;
	case 1:
		m = OMAP_MEM1_RETSTATE_MASK;
		break;
	case 2:
		m = OMAP_MEM2_RETSTATE_MASK;
		break;
	case 3:
		m = OMAP_MEM3_RETSTATE_MASK;
		break;
	case 4:
		m = OMAP_MEM4_RETSTATE_MASK;
	default:
		WARN_ON(1); /* should never happen */
		return -EEXIST;
	}

	return prm_read_mod_bits_shift(pwrdm->prcm_offs,
					pwrstctrl_reg_offs, m);
}

int pwrdm_clear_all_prev_pwrst(struct powerdomain *pwrdm)
{
	if (!pwrdm)
		return -EINVAL;

	/*
	 * XXX should get the powerdomain's current state here;
	 * warn & fail if it is not ON.
	 */

	pr_debug("powerdomain: clearing previous power state reg for %s\n",
		 pwrdm->name);

	prm_write_mod_reg(0, pwrdm->prcm_offs, OMAP3430_PM_PREPWSTST);

	return 0;
}

int pwrdm_enable_hdwr_sar(struct powerdomain *pwrdm)
{
	if (!pwrdm)
		return -EINVAL;

	if (!(pwrdm->flags & PWRDM_HAS_HDWR_SAR))
		return -EINVAL;

	pr_debug("powerdomain: %s: setting SAVEANDRESTORE bit\n",
		 pwrdm->name);

	prm_rmw_mod_reg_bits(0, 1 << OMAP3430ES2_SAVEANDRESTORE_SHIFT,
			     pwrdm->prcm_offs, pwrstctrl_reg_offs);

	return 0;
}

int pwrdm_disable_hdwr_sar(struct powerdomain *pwrdm)
{
	if (!pwrdm)
		return -EINVAL;

	if (!(pwrdm->flags & PWRDM_HAS_HDWR_SAR))
		return -EINVAL;

	pr_debug("powerdomain: %s: clearing SAVEANDRESTORE bit\n",
		 pwrdm->name);

	prm_rmw_mod_reg_bits(1 << OMAP3430ES2_SAVEANDRESTORE_SHIFT, 0,
			     pwrdm->prcm_offs, pwrstctrl_reg_offs);

	return 0;
}

bool pwrdm_has_hdwr_sar(struct powerdomain *pwrdm)
{
	return (pwrdm && pwrdm->flags & PWRDM_HAS_HDWR_SAR) ? 1 : 0;
}

int pwrdm_set_lowpwrstchange(struct powerdomain *pwrdm)
{
	if (!pwrdm)
		return -EINVAL;

	if (!(pwrdm->flags & PWRDM_HAS_LOWPOWERSTATECHANGE))
		return -EINVAL;

	pr_debug("powerdomain: %s: setting LOWPOWERSTATECHANGE bit\n",
		 pwrdm->name);

	prm_rmw_mod_reg_bits(OMAP4430_LOWPOWERSTATECHANGE_MASK,
			     (1 << OMAP4430_LOWPOWERSTATECHANGE_SHIFT),
			     pwrdm->prcm_offs, pwrstctrl_reg_offs);

	return 0;
}

int pwrdm_wait_transition(struct powerdomain *pwrdm)
{
	u32 c = 0;

	if (!pwrdm)
		return -EINVAL;

	/*
	 * REVISIT: pwrdm_wait_transition() may be better implemented
	 * via a callback and a periodic timer check -- how long do we expect
	 * powerdomain transitions to take?
	 */

	/* XXX Is this udelay() value meaningful? */
	while ((prm_read_mod_reg(pwrdm->prcm_offs, pwrstst_reg_offs) &
		OMAP_INTRANSITION_MASK) &&
	       (c++ < PWRDM_TRANSITION_BAILOUT))
			udelay(1);

	if (c > PWRDM_TRANSITION_BAILOUT) {
		printk(KERN_ERR "powerdomain: waited too long for "
		       "powerdomain %s to complete transition\n", pwrdm->name);
		return -EAGAIN;
	}

	pr_debug("powerdomain: completed transition in %d loops\n", c);

	return 0;
}

int pwrdm_state_switch(struct powerdomain *pwrdm)
{
	return _pwrdm_state_switch(pwrdm, PWRDM_STATE_NOW);
}

int pwrdm_clkdm_state_switch(struct clockdomain *clkdm)
{
	if (clkdm != NULL && clkdm->pwrdm.ptr != NULL) {
		pwrdm_wait_transition(clkdm->pwrdm.ptr);
		return pwrdm_state_switch(clkdm->pwrdm.ptr);
	}

	return -EINVAL;
}

int pwrdm_pre_transition(void)
{
	pwrdm_for_each(_pwrdm_pre_transition_cb, NULL);
	return 0;
}

int pwrdm_post_transition(void)
{
	pwrdm_for_each(_pwrdm_post_transition_cb, NULL);
	return 0;
}

