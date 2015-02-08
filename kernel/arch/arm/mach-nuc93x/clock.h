

#include <asm/clkdev.h>

void nuc93x_clk_enable(struct clk *clk, int enable);
void clks_register(struct clk_lookup *clks, size_t num);

struct clk {
	unsigned long		cken;
	unsigned int		enabled;
	void			(*enable)(struct clk *, int enable);
};

#define DEFINE_CLK(_name, _ctrlbit)			\
struct clk clk_##_name = {				\
		.enable	= nuc93x_clk_enable,		\
		.cken	= (1 << _ctrlbit),		\
	}

#define DEF_CLKLOOK(_clk, _devname, _conname)		\
	{						\
		.clk		= _clk,			\
		.dev_id		= _devname,		\
		.con_id		= _conname,		\
	}

