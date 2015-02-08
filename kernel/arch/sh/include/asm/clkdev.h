
#ifndef __ASM_CLKDEV_H
#define __ASM_CLKDEV_H

struct clk;

struct clk_lookup {
	struct list_head	node;
	const char		*dev_id;
	const char		*con_id;
	struct clk		*clk;
};

struct clk_lookup *clkdev_alloc(struct clk *clk, const char *con_id,
	const char *dev_fmt, ...);

void clkdev_add(struct clk_lookup *cl);
void clkdev_drop(struct clk_lookup *cl);

void clkdev_add_table(struct clk_lookup *, size_t);
int clk_add_alias(const char *, const char *, char *, struct device *);

#endif
