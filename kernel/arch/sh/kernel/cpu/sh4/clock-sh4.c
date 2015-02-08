
#include <linux/init.h>
#include <linux/kernel.h>
#include <asm/clock.h>
#include <asm/freq.h>
#include <asm/io.h>

static int ifc_divisors[] = { 1, 2, 3, 4, 6, 8, 1, 1 };
#define bfc_divisors ifc_divisors	/* Same */
static int pfc_divisors[] = { 2, 3, 4, 6, 8, 2, 2, 2 };

static void master_clk_init(struct clk *clk)
{
	clk->rate *= pfc_divisors[__raw_readw(FRQCR) & 0x0007];
}

static struct clk_ops sh4_master_clk_ops = {
	.init		= master_clk_init,
};

static unsigned long module_clk_recalc(struct clk *clk)
{
	int idx = (__raw_readw(FRQCR) & 0x0007);
	return clk->parent->rate / pfc_divisors[idx];
}

static struct clk_ops sh4_module_clk_ops = {
	.recalc		= module_clk_recalc,
};

static unsigned long bus_clk_recalc(struct clk *clk)
{
	int idx = (__raw_readw(FRQCR) >> 3) & 0x0007;
	return clk->parent->rate / bfc_divisors[idx];
}

static struct clk_ops sh4_bus_clk_ops = {
	.recalc		= bus_clk_recalc,
};

static unsigned long cpu_clk_recalc(struct clk *clk)
{
	int idx = (__raw_readw(FRQCR) >> 6) & 0x0007;
	return clk->parent->rate / ifc_divisors[idx];
}

static struct clk_ops sh4_cpu_clk_ops = {
	.recalc		= cpu_clk_recalc,
};

static struct clk_ops *sh4_clk_ops[] = {
	&sh4_master_clk_ops,
	&sh4_module_clk_ops,
	&sh4_bus_clk_ops,
	&sh4_cpu_clk_ops,
};

void __init arch_init_clk_ops(struct clk_ops **ops, int idx)
{
	if (idx < ARRAY_SIZE(sh4_clk_ops))
		*ops = sh4_clk_ops[idx];
}

