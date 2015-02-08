
#undef DEBUG

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/mutex.h>

#include <plat/common.h>
#include <plat/cpu.h>
#include <plat/clockdomain.h>
#include <plat/powerdomain.h>
#include <plat/clock.h>
#include <plat/omap_hwmod.h>

#include "cm.h"

/* Maximum microseconds to wait for OMAP module to reset */
#define MAX_MODULE_RESET_WAIT		10000

/* Name of the OMAP hwmod for the MPU */
#define MPU_INITIATOR_NAME		"mpu"

/* omap_hwmod_list contains all registered struct omap_hwmods */
static LIST_HEAD(omap_hwmod_list);

static DEFINE_MUTEX(omap_hwmod_mutex);

/* mpu_oh: used to add/remove MPU initiator from sleepdep list */
static struct omap_hwmod *mpu_oh;

/* inited: 0 if omap_hwmod_init() has not yet been called; 1 otherwise */
static u8 inited;


/* Private functions */

static int _update_sysc_cache(struct omap_hwmod *oh)
{
	if (!oh->class->sysc) {
		WARN(1, "omap_hwmod: %s: cannot read OCP_SYSCONFIG: not defined on hwmod's class\n", oh->name);
		return -EINVAL;
	}

	/* XXX ensure module interface clock is up */

	oh->_sysc_cache = omap_hwmod_readl(oh, oh->class->sysc->sysc_offs);

	if (!(oh->class->sysc->sysc_flags & SYSC_NO_CACHE))
		oh->_int_flags |= _HWMOD_SYSCONFIG_LOADED;

	return 0;
}

static void _write_sysconfig(u32 v, struct omap_hwmod *oh)
{
	if (!oh->class->sysc) {
		WARN(1, "omap_hwmod: %s: cannot write OCP_SYSCONFIG: not defined on hwmod's class\n", oh->name);
		return;
	}

	/* XXX ensure module interface clock is up */

	if (oh->_sysc_cache != v) {
		oh->_sysc_cache = v;
		omap_hwmod_writel(v, oh, oh->class->sysc->sysc_offs);
	}
}

static int _set_master_standbymode(struct omap_hwmod *oh, u8 standbymode,
				   u32 *v)
{
	u32 mstandby_mask;
	u8 mstandby_shift;

	if (!oh->class->sysc ||
	    !(oh->class->sysc->sysc_flags & SYSC_HAS_MIDLEMODE))
		return -EINVAL;

	if (!oh->class->sysc->sysc_fields) {
		WARN(1, "omap_hwmod: %s: offset struct for sysconfig not provided in class\n", oh->name);
		return -EINVAL;
	}

	mstandby_shift = oh->class->sysc->sysc_fields->midle_shift;
	mstandby_mask = (0x3 << mstandby_shift);

	*v &= ~mstandby_mask;
	*v |= __ffs(standbymode) << mstandby_shift;

	return 0;
}

static int _set_slave_idlemode(struct omap_hwmod *oh, u8 idlemode, u32 *v)
{
	u32 sidle_mask;
	u8 sidle_shift;

	if (!oh->class->sysc ||
	    !(oh->class->sysc->sysc_flags & SYSC_HAS_SIDLEMODE))
		return -EINVAL;

	if (!oh->class->sysc->sysc_fields) {
		WARN(1, "omap_hwmod: %s: offset struct for sysconfig not provided in class\n", oh->name);
		return -EINVAL;
	}

	sidle_shift = oh->class->sysc->sysc_fields->sidle_shift;
	sidle_mask = (0x3 << sidle_shift);

	*v &= ~sidle_mask;
	*v |= __ffs(idlemode) << sidle_shift;

	return 0;
}

static int _set_clockactivity(struct omap_hwmod *oh, u8 clockact, u32 *v)
{
	u32 clkact_mask;
	u8  clkact_shift;

	if (!oh->class->sysc ||
	    !(oh->class->sysc->sysc_flags & SYSC_HAS_CLOCKACTIVITY))
		return -EINVAL;

	if (!oh->class->sysc->sysc_fields) {
		WARN(1, "omap_hwmod: %s: offset struct for sysconfig not provided in class\n", oh->name);
		return -EINVAL;
	}

	clkact_shift = oh->class->sysc->sysc_fields->clkact_shift;
	clkact_mask = (0x3 << clkact_shift);

	*v &= ~clkact_mask;
	*v |= clockact << clkact_shift;

	return 0;
}

static int _set_softreset(struct omap_hwmod *oh, u32 *v)
{
	u32 softrst_mask;

	if (!oh->class->sysc ||
	    !(oh->class->sysc->sysc_flags & SYSC_HAS_SOFTRESET))
		return -EINVAL;

	if (!oh->class->sysc->sysc_fields) {
		WARN(1, "omap_hwmod: %s: offset struct for sysconfig not provided in class\n", oh->name);
		return -EINVAL;
	}

	softrst_mask = (0x1 << oh->class->sysc->sysc_fields->srst_shift);

	*v |= softrst_mask;

	return 0;
}

static int _set_module_autoidle(struct omap_hwmod *oh, u8 autoidle,
				u32 *v)
{
	u32 autoidle_mask;
	u8 autoidle_shift;

	if (!oh->class->sysc ||
	    !(oh->class->sysc->sysc_flags & SYSC_HAS_AUTOIDLE))
		return -EINVAL;

	if (!oh->class->sysc->sysc_fields) {
		WARN(1, "omap_hwmod: %s: offset struct for sysconfig not provided in class\n", oh->name);
		return -EINVAL;
	}

	autoidle_shift = oh->class->sysc->sysc_fields->autoidle_shift;
	autoidle_mask = (0x3 << autoidle_shift);

	*v &= ~autoidle_mask;
	*v |= autoidle << autoidle_shift;

	return 0;
}

static int _enable_wakeup(struct omap_hwmod *oh)
{
	u32 v, wakeup_mask;

	if (!oh->class->sysc ||
	    !(oh->class->sysc->sysc_flags & SYSC_HAS_ENAWAKEUP))
		return -EINVAL;

	if (!oh->class->sysc->sysc_fields) {
		WARN(1, "omap_hwmod: %s: offset struct for sysconfig not provided in class\n", oh->name);
		return -EINVAL;
	}

	wakeup_mask = (0x1 << oh->class->sysc->sysc_fields->enwkup_shift);

	v = oh->_sysc_cache;
	v |= wakeup_mask;
	_write_sysconfig(v, oh);

	/* XXX test pwrdm_get_wken for this hwmod's subsystem */

	oh->_int_flags |= _HWMOD_WAKEUP_ENABLED;

	return 0;
}

static int _disable_wakeup(struct omap_hwmod *oh)
{
	u32 v, wakeup_mask;

	if (!oh->class->sysc ||
	    !(oh->class->sysc->sysc_flags & SYSC_HAS_ENAWAKEUP))
		return -EINVAL;

	if (!oh->class->sysc->sysc_fields) {
		WARN(1, "omap_hwmod: %s: offset struct for sysconfig not provided in class\n", oh->name);
		return -EINVAL;
	}

	wakeup_mask = (0x1 << oh->class->sysc->sysc_fields->enwkup_shift);

	v = oh->_sysc_cache;
	v &= ~wakeup_mask;
	_write_sysconfig(v, oh);

	/* XXX test pwrdm_get_wken for this hwmod's subsystem */

	oh->_int_flags &= ~_HWMOD_WAKEUP_ENABLED;

	return 0;
}

static int _add_initiator_dep(struct omap_hwmod *oh, struct omap_hwmod *init_oh)
{
	if (!oh->_clk)
		return -EINVAL;

	return clkdm_add_sleepdep(oh->_clk->clkdm, init_oh->_clk->clkdm);
}

static int _del_initiator_dep(struct omap_hwmod *oh, struct omap_hwmod *init_oh)
{
	if (!oh->_clk)
		return -EINVAL;

	return clkdm_del_sleepdep(oh->_clk->clkdm, init_oh->_clk->clkdm);
}

static int _init_main_clk(struct omap_hwmod *oh)
{
	int ret = 0;

	if (!oh->main_clk)
		return 0;

	oh->_clk = omap_clk_get_by_name(oh->main_clk);
	if (!oh->_clk) {
		pr_warning("omap_hwmod: %s: cannot clk_get main_clk %s\n",
			   oh->name, oh->main_clk);
		return -EINVAL;
	}

	if (!oh->_clk->clkdm)
		pr_warning("omap_hwmod: %s: missing clockdomain for %s.\n",
			   oh->main_clk, oh->_clk->name);

	return ret;
}

static int _init_interface_clks(struct omap_hwmod *oh)
{
	struct clk *c;
	int i;
	int ret = 0;

	if (oh->slaves_cnt == 0)
		return 0;

	for (i = 0; i < oh->slaves_cnt; i++) {
		struct omap_hwmod_ocp_if *os = oh->slaves[i];

		if (!os->clk)
			continue;

		c = omap_clk_get_by_name(os->clk);
		if (!c) {
			pr_warning("omap_hwmod: %s: cannot clk_get interface_clk %s\n",
				   oh->name, os->clk);
			ret = -EINVAL;
		}
		os->_clk = c;
	}

	return ret;
}

static int _init_opt_clks(struct omap_hwmod *oh)
{
	struct omap_hwmod_opt_clk *oc;
	struct clk *c;
	int i;
	int ret = 0;

	for (i = oh->opt_clks_cnt, oc = oh->opt_clks; i > 0; i--, oc++) {
		c = omap_clk_get_by_name(oc->clk);
		if (!c) {
			pr_warning("omap_hwmod: %s: cannot clk_get opt_clk %s\n",
				   oh->name, oc->clk);
			ret = -EINVAL;
		}
		oc->_clk = c;
	}

	return ret;
}

static int _enable_clocks(struct omap_hwmod *oh)
{
	int i;

	pr_debug("omap_hwmod: %s: enabling clocks\n", oh->name);

	if (oh->_clk)
		clk_enable(oh->_clk);

	if (oh->slaves_cnt > 0) {
		for (i = 0; i < oh->slaves_cnt; i++) {
			struct omap_hwmod_ocp_if *os = oh->slaves[i];
			struct clk *c = os->_clk;

			if (c && (os->flags & OCPIF_SWSUP_IDLE))
				clk_enable(c);
		}
	}

	/* The opt clocks are controlled by the device driver. */

	return 0;
}

static int _disable_clocks(struct omap_hwmod *oh)
{
	int i;

	pr_debug("omap_hwmod: %s: disabling clocks\n", oh->name);

	if (oh->_clk)
		clk_disable(oh->_clk);

	if (oh->slaves_cnt > 0) {
		for (i = 0; i < oh->slaves_cnt; i++) {
			struct omap_hwmod_ocp_if *os = oh->slaves[i];
			struct clk *c = os->_clk;

			if (c && (os->flags & OCPIF_SWSUP_IDLE))
				clk_disable(c);
		}
	}

	/* The opt clocks are controlled by the device driver. */

	return 0;
}

static int _find_mpu_port_index(struct omap_hwmod *oh)
{
	int i;
	int found = 0;

	if (!oh || oh->slaves_cnt == 0)
		return -EINVAL;

	for (i = 0; i < oh->slaves_cnt; i++) {
		struct omap_hwmod_ocp_if *os = oh->slaves[i];

		if (os->user & OCP_USER_MPU) {
			found = 1;
			break;
		}
	}

	if (found)
		pr_debug("omap_hwmod: %s: MPU OCP slave port ID  %d\n",
			 oh->name, i);
	else
		pr_debug("omap_hwmod: %s: no MPU OCP slave port found\n",
			 oh->name);

	return (found) ? i : -EINVAL;
}

static void __iomem *_find_mpu_rt_base(struct omap_hwmod *oh, u8 index)
{
	struct omap_hwmod_ocp_if *os;
	struct omap_hwmod_addr_space *mem;
	int i;
	int found = 0;
	void __iomem *va_start;

	if (!oh || oh->slaves_cnt == 0)
		return NULL;

	os = oh->slaves[index];

	for (i = 0, mem = os->addr; i < os->addr_cnt; i++, mem++) {
		if (mem->flags & ADDR_TYPE_RT) {
			found = 1;
			break;
		}
	}

	if (found) {
		va_start = ioremap(mem->pa_start, mem->pa_end - mem->pa_start);
		if (!va_start) {
			pr_err("omap_hwmod: %s: Could not ioremap\n", oh->name);
			return NULL;
		}
		pr_debug("omap_hwmod: %s: MPU register target at va %p\n",
			 oh->name, va_start);
	} else {
		pr_debug("omap_hwmod: %s: no MPU register target found\n",
			 oh->name);
	}

	return (found) ? va_start : NULL;
}

static void _sysc_enable(struct omap_hwmod *oh)
{
	u8 idlemode, sf;
	u32 v;

	if (!oh->class->sysc)
		return;

	v = oh->_sysc_cache;
	sf = oh->class->sysc->sysc_flags;

	if (sf & SYSC_HAS_SIDLEMODE) {
		idlemode = (oh->flags & HWMOD_SWSUP_SIDLE) ?
			HWMOD_IDLEMODE_NO : HWMOD_IDLEMODE_SMART;
		_set_slave_idlemode(oh, idlemode, &v);
	}

	if (sf & SYSC_HAS_MIDLEMODE) {
		idlemode = (oh->flags & HWMOD_SWSUP_MSTANDBY) ?
			HWMOD_IDLEMODE_NO : HWMOD_IDLEMODE_SMART;
		_set_master_standbymode(oh, idlemode, &v);
	}

	if (sf & SYSC_HAS_AUTOIDLE) {
		idlemode = (oh->flags & HWMOD_NO_OCP_AUTOIDLE) ?
			0 : 1;
		_set_module_autoidle(oh, idlemode, &v);
	}

	/* XXX OCP ENAWAKEUP bit? */

	/*
	 * XXX The clock framework should handle this, by
	 * calling into this code.  But this must wait until the
	 * clock structures are tagged with omap_hwmod entries
	 */
	if ((oh->flags & HWMOD_SET_DEFAULT_CLOCKACT) &&
	    (sf & SYSC_HAS_CLOCKACTIVITY))
		_set_clockactivity(oh, oh->class->sysc->clockact, &v);

	_write_sysconfig(v, oh);
}

static void _sysc_idle(struct omap_hwmod *oh)
{
	u8 idlemode, sf;
	u32 v;

	if (!oh->class->sysc)
		return;

	v = oh->_sysc_cache;
	sf = oh->class->sysc->sysc_flags;

	if (sf & SYSC_HAS_SIDLEMODE) {
		idlemode = (oh->flags & HWMOD_SWSUP_SIDLE) ?
			HWMOD_IDLEMODE_FORCE : HWMOD_IDLEMODE_SMART;
		_set_slave_idlemode(oh, idlemode, &v);
	}

	if (sf & SYSC_HAS_MIDLEMODE) {
		idlemode = (oh->flags & HWMOD_SWSUP_MSTANDBY) ?
			HWMOD_IDLEMODE_FORCE : HWMOD_IDLEMODE_SMART;
		_set_master_standbymode(oh, idlemode, &v);
	}

	_write_sysconfig(v, oh);
}

static void _sysc_shutdown(struct omap_hwmod *oh)
{
	u32 v;
	u8 sf;

	if (!oh->class->sysc)
		return;

	v = oh->_sysc_cache;
	sf = oh->class->sysc->sysc_flags;

	if (sf & SYSC_HAS_SIDLEMODE)
		_set_slave_idlemode(oh, HWMOD_IDLEMODE_FORCE, &v);

	if (sf & SYSC_HAS_MIDLEMODE)
		_set_master_standbymode(oh, HWMOD_IDLEMODE_FORCE, &v);

	if (sf & SYSC_HAS_AUTOIDLE)
		_set_module_autoidle(oh, 1, &v);

	_write_sysconfig(v, oh);
}

static struct omap_hwmod *_lookup(const char *name)
{
	struct omap_hwmod *oh, *temp_oh;

	oh = NULL;

	list_for_each_entry(temp_oh, &omap_hwmod_list, node) {
		if (!strcmp(name, temp_oh->name)) {
			oh = temp_oh;
			break;
		}
	}

	return oh;
}

static int _init_clocks(struct omap_hwmod *oh)
{
	int ret = 0;

	if (!oh || (oh->_state != _HWMOD_STATE_REGISTERED))
		return -EINVAL;

	pr_debug("omap_hwmod: %s: looking up clocks\n", oh->name);

	ret |= _init_main_clk(oh);
	ret |= _init_interface_clks(oh);
	ret |= _init_opt_clks(oh);

	if (!ret)
		oh->_state = _HWMOD_STATE_CLKS_INITED;

	return 0;
}

static int _wait_target_ready(struct omap_hwmod *oh)
{
	struct omap_hwmod_ocp_if *os;
	int ret;

	if (!oh)
		return -EINVAL;

	if (oh->_int_flags & _HWMOD_NO_MPU_PORT)
		return 0;

	os = oh->slaves[oh->_mpu_port_index];

	if (oh->flags & HWMOD_NO_IDLEST)
		return 0;

	/* XXX check module SIDLEMODE */

	/* XXX check clock enable states */

	if (cpu_is_omap24xx() || cpu_is_omap34xx()) {
		ret = omap2_cm_wait_module_ready(oh->prcm.omap2.module_offs,
						 oh->prcm.omap2.idlest_reg_id,
						 oh->prcm.omap2.idlest_idle_bit);
	} else if (cpu_is_omap44xx()) {
		ret = omap4_cm_wait_module_ready(oh->prcm.omap4.clkctrl_reg);
	} else {
		BUG();
	};

	return ret;
}

static int _reset(struct omap_hwmod *oh)
{
	u32 r, v;
	int c = 0;

	if (!oh->class->sysc ||
	    !(oh->class->sysc->sysc_flags & SYSC_HAS_SOFTRESET) ||
	    (oh->class->sysc->sysc_flags & SYSS_MISSING))
		return -EINVAL;

	/* clocks must be on for this operation */
	if (oh->_state != _HWMOD_STATE_ENABLED) {
		WARN(1, "omap_hwmod: %s: reset can only be entered from "
		     "enabled state\n", oh->name);
		return -EINVAL;
	}

	pr_debug("omap_hwmod: %s: resetting\n", oh->name);

	v = oh->_sysc_cache;
	r = _set_softreset(oh, &v);
	if (r)
		return r;
	_write_sysconfig(v, oh);

	omap_test_timeout((omap_hwmod_readl(oh, oh->class->sysc->syss_offs) &
			   SYSS_RESETDONE_MASK),
			  MAX_MODULE_RESET_WAIT, c);

	if (c == MAX_MODULE_RESET_WAIT)
		WARN(1, "omap_hwmod: %s: failed to reset in %d usec\n",
		     oh->name, MAX_MODULE_RESET_WAIT);
	else
		pr_debug("omap_hwmod: %s: reset in %d usec\n", oh->name, c);

	/*
	 * XXX add _HWMOD_STATE_WEDGED for modules that don't come back from
	 * _wait_target_ready() or _reset()
	 */

	return (c == MAX_MODULE_RESET_WAIT) ? -ETIMEDOUT : 0;
}

static int _enable(struct omap_hwmod *oh)
{
	int r;

	if (oh->_state != _HWMOD_STATE_INITIALIZED &&
	    oh->_state != _HWMOD_STATE_IDLE &&
	    oh->_state != _HWMOD_STATE_DISABLED) {
		WARN(1, "omap_hwmod: %s: enabled state can only be entered "
		     "from initialized, idle, or disabled state\n", oh->name);
		return -EINVAL;
	}

	pr_debug("omap_hwmod: %s: enabling\n", oh->name);

	/* XXX mux balls */

	_add_initiator_dep(oh, mpu_oh);
	_enable_clocks(oh);

	r = _wait_target_ready(oh);
	if (!r) {
		oh->_state = _HWMOD_STATE_ENABLED;

		/* Access the sysconfig only if the target is ready */
		if (oh->class->sysc) {
			if (!(oh->_int_flags & _HWMOD_SYSCONFIG_LOADED))
				_update_sysc_cache(oh);
			_sysc_enable(oh);
		}
	} else {
		pr_debug("omap_hwmod: %s: _wait_target_ready: %d\n",
			 oh->name, r);
	}

	return r;
}

static int _idle(struct omap_hwmod *oh)
{
	if (oh->_state != _HWMOD_STATE_ENABLED) {
		WARN(1, "omap_hwmod: %s: idle state can only be entered from "
		     "enabled state\n", oh->name);
		return -EINVAL;
	}

	pr_debug("omap_hwmod: %s: idling\n", oh->name);

	if (oh->class->sysc)
		_sysc_idle(oh);
	_del_initiator_dep(oh, mpu_oh);
	_disable_clocks(oh);

	oh->_state = _HWMOD_STATE_IDLE;

	return 0;
}

static int _shutdown(struct omap_hwmod *oh)
{
	if (oh->_state != _HWMOD_STATE_IDLE &&
	    oh->_state != _HWMOD_STATE_ENABLED) {
		WARN(1, "omap_hwmod: %s: disabled state can only be entered "
		     "from idle, or enabled state\n", oh->name);
		return -EINVAL;
	}

	pr_debug("omap_hwmod: %s: disabling\n", oh->name);

	if (oh->class->sysc)
		_sysc_shutdown(oh);
	_del_initiator_dep(oh, mpu_oh);
	/* XXX what about the other system initiators here? DMA, tesla, d2d */
	_disable_clocks(oh);
	/* XXX Should this code also force-disable the optional clocks? */

	/* XXX mux any associated balls to safe mode */

	oh->_state = _HWMOD_STATE_DISABLED;

	return 0;
}

static int _setup(struct omap_hwmod *oh)
{
	int i, r;

	if (!oh)
		return -EINVAL;

	/* Set iclk autoidle mode */
	if (oh->slaves_cnt > 0) {
		for (i = 0; i < oh->slaves_cnt; i++) {
			struct omap_hwmod_ocp_if *os = oh->slaves[i];
			struct clk *c = os->_clk;

			if (!c)
				continue;

			if (os->flags & OCPIF_SWSUP_IDLE) {
				/* XXX omap_iclk_deny_idle(c); */
			} else {
				/* XXX omap_iclk_allow_idle(c); */
				clk_enable(c);
			}
		}
	}

	oh->_state = _HWMOD_STATE_INITIALIZED;

	r = _enable(oh);
	if (r) {
		pr_warning("omap_hwmod: %s: cannot be enabled (%d)\n",
			   oh->name, oh->_state);
		return 0;
	}

	if (!(oh->flags & HWMOD_INIT_NO_RESET)) {
		/*
		 * XXX Do the OCP_SYSCONFIG bits need to be
		 * reprogrammed after a reset?  If not, then this can
		 * be removed.  If they do, then probably the
		 * _enable() function should be split to avoid the
		 * rewrite of the OCP_SYSCONFIG register.
		 */
		if (oh->class->sysc) {
			_update_sysc_cache(oh);
			_sysc_enable(oh);
		}
	}

	if (!(oh->flags & HWMOD_INIT_NO_IDLE))
		_idle(oh);

	return 0;
}



/* Public functions */

u32 omap_hwmod_readl(struct omap_hwmod *oh, u16 reg_offs)
{
	return __raw_readl(oh->_rt_va + reg_offs);
}

void omap_hwmod_writel(u32 v, struct omap_hwmod *oh, u16 reg_offs)
{
	__raw_writel(v, oh->_rt_va + reg_offs);
}

int omap_hwmod_set_slave_idlemode(struct omap_hwmod *oh, u8 idlemode)
{
	u32 v;
	int retval = 0;

	if (!oh)
		return -EINVAL;

	v = oh->_sysc_cache;

	retval = _set_slave_idlemode(oh, idlemode, &v);
	if (!retval)
		_write_sysconfig(v, oh);

	return retval;
}

int omap_hwmod_register(struct omap_hwmod *oh)
{
	int ret, ms_id;

	if (!oh || !oh->name || !oh->class || !oh->class->name ||
	    (oh->_state != _HWMOD_STATE_UNKNOWN))
		return -EINVAL;

	mutex_lock(&omap_hwmod_mutex);

	pr_debug("omap_hwmod: %s: registering\n", oh->name);

	if (_lookup(oh->name)) {
		ret = -EEXIST;
		goto ohr_unlock;
	}

	ms_id = _find_mpu_port_index(oh);
	if (!IS_ERR_VALUE(ms_id)) {
		oh->_mpu_port_index = ms_id;
		oh->_rt_va = _find_mpu_rt_base(oh, oh->_mpu_port_index);
	} else {
		oh->_int_flags |= _HWMOD_NO_MPU_PORT;
	}

	list_add_tail(&oh->node, &omap_hwmod_list);

	oh->_state = _HWMOD_STATE_REGISTERED;

	ret = 0;

ohr_unlock:
	mutex_unlock(&omap_hwmod_mutex);
	return ret;
}

struct omap_hwmod *omap_hwmod_lookup(const char *name)
{
	struct omap_hwmod *oh;

	if (!name)
		return NULL;

	mutex_lock(&omap_hwmod_mutex);
	oh = _lookup(name);
	mutex_unlock(&omap_hwmod_mutex);

	return oh;
}

int omap_hwmod_for_each(int (*fn)(struct omap_hwmod *oh))
{
	struct omap_hwmod *temp_oh;
	int ret;

	if (!fn)
		return -EINVAL;

	mutex_lock(&omap_hwmod_mutex);
	list_for_each_entry(temp_oh, &omap_hwmod_list, node) {
		ret = (*fn)(temp_oh);
		if (ret)
			break;
	}
	mutex_unlock(&omap_hwmod_mutex);

	return ret;
}


int omap_hwmod_init(struct omap_hwmod **ohs)
{
	struct omap_hwmod *oh;
	int r;

	if (inited)
		return -EINVAL;

	inited = 1;

	if (!ohs)
		return 0;

	oh = *ohs;
	while (oh) {
		if (omap_chip_is(oh->omap_chip)) {
			r = omap_hwmod_register(oh);
			WARN(r, "omap_hwmod: %s: omap_hwmod_register returned "
			     "%d\n", oh->name, r);
		}
		oh = *++ohs;
	}

	return 0;
}

int omap_hwmod_late_init(void)
{
	int r;

	/* XXX check return value */
	r = omap_hwmod_for_each(_init_clocks);
	WARN(r, "omap_hwmod: omap_hwmod_late_init(): _init_clocks failed\n");

	mpu_oh = omap_hwmod_lookup(MPU_INITIATOR_NAME);
	WARN(!mpu_oh, "omap_hwmod: could not find MPU initiator hwmod %s\n",
	     MPU_INITIATOR_NAME);

	omap_hwmod_for_each(_setup);

	return 0;
}

int omap_hwmod_unregister(struct omap_hwmod *oh)
{
	if (!oh)
		return -EINVAL;

	pr_debug("omap_hwmod: %s: unregistering\n", oh->name);

	mutex_lock(&omap_hwmod_mutex);
	iounmap(oh->_rt_va);
	list_del(&oh->node);
	mutex_unlock(&omap_hwmod_mutex);

	return 0;
}

int omap_hwmod_enable(struct omap_hwmod *oh)
{
	int r;

	if (!oh)
		return -EINVAL;

	mutex_lock(&omap_hwmod_mutex);
	r = _enable(oh);
	mutex_unlock(&omap_hwmod_mutex);

	return r;
}

int omap_hwmod_idle(struct omap_hwmod *oh)
{
	if (!oh)
		return -EINVAL;

	mutex_lock(&omap_hwmod_mutex);
	_idle(oh);
	mutex_unlock(&omap_hwmod_mutex);

	return 0;
}

int omap_hwmod_shutdown(struct omap_hwmod *oh)
{
	if (!oh)
		return -EINVAL;

	mutex_lock(&omap_hwmod_mutex);
	_shutdown(oh);
	mutex_unlock(&omap_hwmod_mutex);

	return 0;
}

int omap_hwmod_enable_clocks(struct omap_hwmod *oh)
{
	mutex_lock(&omap_hwmod_mutex);
	_enable_clocks(oh);
	mutex_unlock(&omap_hwmod_mutex);

	return 0;
}

int omap_hwmod_disable_clocks(struct omap_hwmod *oh)
{
	mutex_lock(&omap_hwmod_mutex);
	_disable_clocks(oh);
	mutex_unlock(&omap_hwmod_mutex);

	return 0;
}

void omap_hwmod_ocp_barrier(struct omap_hwmod *oh)
{
	BUG_ON(!oh);

	if (!oh->class->sysc || !oh->class->sysc->sysc_flags) {
		WARN(1, "omap_device: %s: OCP barrier impossible due to "
		      "device configuration\n", oh->name);
		return;
	}

	/*
	 * Forces posted writes to complete on the OCP thread handling
	 * register writes
	 */
	omap_hwmod_readl(oh, oh->class->sysc->sysc_offs);
}

int omap_hwmod_reset(struct omap_hwmod *oh)
{
	int r;

	if (!oh || !(oh->_state & _HWMOD_STATE_ENABLED))
		return -EINVAL;

	mutex_lock(&omap_hwmod_mutex);
	r = _reset(oh);
	if (!r)
		r = _enable(oh);
	mutex_unlock(&omap_hwmod_mutex);

	return r;
}

int omap_hwmod_count_resources(struct omap_hwmod *oh)
{
	int ret, i;

	ret = oh->mpu_irqs_cnt + oh->sdma_chs_cnt;

	for (i = 0; i < oh->slaves_cnt; i++)
		ret += oh->slaves[i]->addr_cnt;

	return ret;
}

int omap_hwmod_fill_resources(struct omap_hwmod *oh, struct resource *res)
{
	int i, j;
	int r = 0;

	/* For each IRQ, DMA, memory area, fill in array.*/

	for (i = 0; i < oh->mpu_irqs_cnt; i++) {
		(res + r)->name = (oh->mpu_irqs + i)->name;
		(res + r)->start = (oh->mpu_irqs + i)->irq;
		(res + r)->end = (oh->mpu_irqs + i)->irq;
		(res + r)->flags = IORESOURCE_IRQ;
		r++;
	}

	for (i = 0; i < oh->sdma_chs_cnt; i++) {
		(res + r)->name = (oh->sdma_chs + i)->name;
		(res + r)->start = (oh->sdma_chs + i)->dma_ch;
		(res + r)->end = (oh->sdma_chs + i)->dma_ch;
		(res + r)->flags = IORESOURCE_DMA;
		r++;
	}

	for (i = 0; i < oh->slaves_cnt; i++) {
		struct omap_hwmod_ocp_if *os;

		os = oh->slaves[i];

		for (j = 0; j < os->addr_cnt; j++) {
			(res + r)->start = (os->addr + j)->pa_start;
			(res + r)->end = (os->addr + j)->pa_end;
			(res + r)->flags = IORESOURCE_MEM;
			r++;
		}
	}

	return r;
}

struct powerdomain *omap_hwmod_get_pwrdm(struct omap_hwmod *oh)
{
	struct clk *c;

	if (!oh)
		return NULL;

	if (oh->_clk) {
		c = oh->_clk;
	} else {
		if (oh->_int_flags & _HWMOD_NO_MPU_PORT)
			return NULL;
		c = oh->slaves[oh->_mpu_port_index]->_clk;
	}

	if (!c->clkdm)
		return NULL;

	return c->clkdm->pwrdm.ptr;

}

int omap_hwmod_add_initiator_dep(struct omap_hwmod *oh,
				 struct omap_hwmod *init_oh)
{
	return _add_initiator_dep(oh, init_oh);
}


int omap_hwmod_del_initiator_dep(struct omap_hwmod *oh,
				 struct omap_hwmod *init_oh)
{
	return _del_initiator_dep(oh, init_oh);
}

int omap_hwmod_enable_wakeup(struct omap_hwmod *oh)
{
	if (!oh->class->sysc ||
	    !(oh->class->sysc->sysc_flags & SYSC_HAS_ENAWAKEUP))
		return -EINVAL;

	mutex_lock(&omap_hwmod_mutex);
	_enable_wakeup(oh);
	mutex_unlock(&omap_hwmod_mutex);

	return 0;
}

int omap_hwmod_disable_wakeup(struct omap_hwmod *oh)
{
	if (!oh->class->sysc ||
	    !(oh->class->sysc->sysc_flags & SYSC_HAS_ENAWAKEUP))
		return -EINVAL;

	mutex_lock(&omap_hwmod_mutex);
	_disable_wakeup(oh);
	mutex_unlock(&omap_hwmod_mutex);

	return 0;
}

int omap_hwmod_for_each_by_class(const char *classname,
				 int (*fn)(struct omap_hwmod *oh,
					   void *user),
				 void *user)
{
	struct omap_hwmod *temp_oh;
	int ret = 0;

	if (!classname || !fn)
		return -EINVAL;

	pr_debug("omap_hwmod: %s: looking for modules of class %s\n",
		 __func__, classname);

	mutex_lock(&omap_hwmod_mutex);

	list_for_each_entry(temp_oh, &omap_hwmod_list, node) {
		if (!strcmp(temp_oh->class->name, classname)) {
			pr_debug("omap_hwmod: %s: %s: calling callback fn\n",
				 __func__, temp_oh->name);
			ret = (*fn)(temp_oh, user);
			if (ret)
				break;
		}
	}

	mutex_unlock(&omap_hwmod_mutex);

	if (ret)
		pr_debug("omap_hwmod: %s: iterator terminated early: %d\n",
			 __func__, ret);

	return ret;
}

