

#ifndef __MACH_CLOCK_H
#define __MACH_CLOCK_H

#include <linux/clk.h>

struct clk {
	struct list_head node;
	struct module *owner;
	struct device *dev;
	const char *name;
	struct clk *parent;

	spinlock_t lock;
	unsigned long rate;
	bool reset;
	__u16 clk_val;
	__s8 usecount;
	__u32 res_reg;
	__u16 res_mask;

	bool hw_ctrld;

	void (*recalc) (struct clk *);
	int (*set_rate) (struct clk *, unsigned long);
	unsigned long (*get_rate) (struct clk *);
	unsigned long (*round_rate) (struct clk *, unsigned long);
	void (*init) (struct clk *);
	void (*enable) (struct clk *);
	void (*disable) (struct clk *);
};

void u300_clock_primecells(void);
void u300_unclock_primecells(void);
void u300_enable_intcon_clock(void);
void u300_enable_timer_clock(void);

#endif
