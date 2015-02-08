

struct clksrc_sources {
	unsigned int	nr_sources;
	struct clk	**sources;
};

struct clksrc_reg {
	void __iomem		*reg;
	unsigned short		shift;
	unsigned short		size;
};

struct clksrc_clk {
	struct clk		clk;
	struct clksrc_sources	*sources;

	struct clksrc_reg	reg_src;
	struct clksrc_reg	reg_div;
};

extern void s3c_set_clksrc(struct clksrc_clk *clk, bool announce);

extern void s3c_register_clksrc(struct clksrc_clk *srcs, int size);
