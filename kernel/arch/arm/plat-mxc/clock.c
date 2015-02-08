

/* #define DEBUG */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/semaphore.h>
#include <linux/string.h>

#include <mach/clock.h>
#include <mach/hardware.h>

static LIST_HEAD(clocks);
static DEFINE_MUTEX(clocks_mutex);


static void __clk_disable(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return;

	__clk_disable(clk->parent);
	__clk_disable(clk->secondary);

	WARN_ON(!clk->usecount);
	if (!(--clk->usecount) && clk->disable)
		clk->disable(clk);
}

static int __clk_enable(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	__clk_enable(clk->parent);
	__clk_enable(clk->secondary);

	if (clk->usecount++ == 0 && clk->enable)
		clk->enable(clk);

	return 0;
}

int clk_enable(struct clk *clk)
{
	int ret = 0;

	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	mutex_lock(&clocks_mutex);
	ret = __clk_enable(clk);
	mutex_unlock(&clocks_mutex);

	return ret;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return;

	mutex_lock(&clocks_mutex);
	__clk_disable(clk);
	mutex_unlock(&clocks_mutex);
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return 0UL;

	if (clk->get_rate)
		return clk->get_rate(clk);

	return clk_get_rate(clk->parent);
}
EXPORT_SYMBOL(clk_get_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (clk == NULL || IS_ERR(clk) || !clk->round_rate)
		return 0;

	return clk->round_rate(clk, rate);
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = -EINVAL;

	if (clk == NULL || IS_ERR(clk) || clk->set_rate == NULL || rate == 0)
		return ret;

	mutex_lock(&clocks_mutex);
	ret = clk->set_rate(clk, rate);
	mutex_unlock(&clocks_mutex);

	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

/* Set the clock's parent to another clock source */
int clk_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = -EINVAL;

	if (clk == NULL || IS_ERR(clk) || parent == NULL ||
	    IS_ERR(parent) || clk->set_parent == NULL)
		return ret;

	mutex_lock(&clocks_mutex);
	ret = clk->set_parent(clk, parent);
	if (ret == 0)
		clk->parent = parent;
	mutex_unlock(&clocks_mutex);

	return ret;
}
EXPORT_SYMBOL(clk_set_parent);

/* Retrieve the clock's parent clock source */
struct clk *clk_get_parent(struct clk *clk)
{
	struct clk *ret = NULL;

	if (clk == NULL || IS_ERR(clk))
		return ret;

	return clk->parent;
}
EXPORT_SYMBOL(clk_get_parent);

unsigned long mxc_decode_pll(unsigned int reg_val, u32 freq)
{
	long long ll;
	int mfn_abs;
	unsigned int mfi, mfn, mfd, pd;

	mfi = (reg_val >> 10) & 0xf;
	mfn = reg_val & 0x3ff;
	mfd = (reg_val >> 16) & 0x3ff;
	pd =  (reg_val >> 26) & 0xf;

	mfi = mfi <= 5 ? 5 : mfi;

	mfn_abs = mfn;

	/* On all i.MXs except i.MX1 and i.MX21 mfn is a 10bit
	 * 2's complements number
	 */
	if (!cpu_is_mx1() && !cpu_is_mx21() && mfn >= 0x200)
		mfn_abs = 0x400 - mfn;

	freq *= 2;
	freq /= pd + 1;

	ll = (unsigned long long)freq * mfn_abs;

	do_div(ll, mfd + 1);

	if (!cpu_is_mx1() && !cpu_is_mx21() && mfn >= 0x200)
		ll = -ll;

	ll = (freq * mfi) + ll;

	return ll;
}
