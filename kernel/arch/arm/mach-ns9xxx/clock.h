
#ifndef __NS9XXX_CLOCK_H
#define __NS9XXX_CLOCK_H

#include <linux/list.h>

struct clk {
	struct module *owner;
	const char *name;
	int id;

	struct clk *parent;

	unsigned long rate;
	int (*endisable)(struct clk *, int enable);
	unsigned long (*get_rate)(struct clk *);

	struct list_head node;
	unsigned long refcount;
	unsigned long usage;
};

int clk_register(struct clk *clk);
int clk_unregister(struct clk *clk);

#endif /* ifndef __NS9XXX_CLOCK_H */
