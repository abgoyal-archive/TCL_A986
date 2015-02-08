
#ifndef __MACH_CLK_H
#define __MACH_CLK_H

#define MSM_AXI_MAX_FREQ	LONG_MAX

enum clk_reset_action {
	CLK_RESET_DEASSERT	= 0,
	CLK_RESET_ASSERT	= 1
};

struct clk;

/* Rate is minimum clock rate in Hz */
int clk_set_min_rate(struct clk *clk, unsigned long rate);

/* Rate is maximum clock rate in Hz */
int clk_set_max_rate(struct clk *clk, unsigned long rate);

/* Assert/Deassert reset to a hardware block associated with a clock */
int clk_reset(struct clk *clk, enum clk_reset_action action);

/* Set clock-specific configuration parameters */
int clk_set_flags(struct clk *clk, unsigned long flags);

#endif
