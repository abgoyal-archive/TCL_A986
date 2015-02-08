

#ifndef __PLAT_CLOCK_H
#define __PLAT_CLOCK_H

#include <linux/list.h>
#include <asm/clkdev.h>
#include <linux/types.h>

/* clk structure flags */
#define	ALWAYS_ENABLED		(1 << 0) /* clock always enabled */
#define	RESET_TO_ENABLE		(1 << 1) /* reset register bit to enable clk */

struct clkops {
	int (*enable) (struct clk *);
	void (*disable) (struct clk *);
};

struct pclk_info {
	struct clk *pclk;
	u8 pclk_mask;
	u8 scalable;
};

struct pclk_sel {
	struct pclk_info *pclk_info;
	u8 pclk_count;
	unsigned int *pclk_sel_reg;
	unsigned int pclk_sel_mask;
};

struct clk {
	unsigned int usage_count;
	unsigned int flags;
	unsigned long rate;
	unsigned int *en_reg;
	u8 en_reg_bit;
	const struct clkops *ops;
	void (*recalc) (struct clk *);

	struct clk *pclk;
	struct pclk_sel *pclk_sel;
	unsigned int pclk_sel_shift;

	struct list_head children;
	struct list_head sibling;
	void *private_data;
};

/* pll configuration structure */
struct pll_clk_config {
	unsigned int *mode_reg;
	unsigned int *cfg_reg;
};

/* ahb and apb bus configuration structure */
struct bus_clk_config {
	unsigned int *reg;
	unsigned int mask;
	unsigned int shift;
};

struct aux_clk_config {
	unsigned int *synth_reg;
};

/* platform specific clock functions */
void clk_register(struct clk_lookup *cl);
void recalc_root_clocks(void);

/* clock recalc functions */
void follow_parent(struct clk *clk);
void pll1_clk_recalc(struct clk *clk);
void bus_clk_recalc(struct clk *clk);
void gpt_clk_recalc(struct clk *clk);
void aux_clk_recalc(struct clk *clk);

#endif /* __PLAT_CLOCK_H */
