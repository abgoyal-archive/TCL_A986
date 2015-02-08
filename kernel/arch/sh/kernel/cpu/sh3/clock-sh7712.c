
#include <linux/init.h>
#include <linux/kernel.h>
#include <asm/clock.h>
#include <asm/freq.h>
#include <asm/io.h>

static int multipliers[] = { 1, 2, 3 };
static int divisors[]    = { 1, 2, 3, 4, 6 };

static void master_clk_init(struct clk *clk)
{
	int frqcr = __raw_readw(FRQCR);
	int idx = (frqcr & 0x0300) >> 8;

	clk->rate *= multipliers[idx];
}

static struct clk_ops sh7712_master_clk_ops = {
	.init		= master_clk_init,
};

static unsigned long module_clk_recalc(struct clk *clk)
{
	int frqcr = __raw_readw(FRQCR);
	int idx = frqcr & 0x0007;

	return clk->parent->rate / divisors[idx];
}

static struct clk_ops sh7712_module_clk_ops = {
	.recalc		= module_clk_recalc,
};

static unsigned long cpu_clk_recalc(struct clk *clk)
{
	int frqcr = __raw_readw(FRQCR);
	int idx = (frqcr & 0x0030) >> 4;

	return clk->parent->rate / divisors[idx];
}

static struct clk_ops sh7712_cpu_clk_ops = {
	.recalc		= cpu_clk_recalc,
};

static struct clk_ops *sh7712_clk_ops[] = {
	&sh7712_master_clk_ops,
	&sh7712_module_clk_ops,
	&sh7712_cpu_clk_ops,
};

void __init arch_init_clk_ops(struct clk_ops **ops, int idx)
{
	if (idx < ARRAY_SIZE(sh7712_clk_ops))
		*ops = sh7712_clk_ops[idx];
}

