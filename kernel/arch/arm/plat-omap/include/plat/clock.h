

#ifndef __ARCH_ARM_OMAP_CLOCK_H
#define __ARCH_ARM_OMAP_CLOCK_H

#include <linux/list.h>

struct module;
struct clk;
struct clockdomain;

struct clkops {
	int			(*enable)(struct clk *);
	void			(*disable)(struct clk *);
	void			(*find_idlest)(struct clk *, void __iomem **,
					       u8 *, u8 *);
	void			(*find_companion)(struct clk *, void __iomem **,
						  u8 *);
};

#ifdef CONFIG_ARCH_OMAP2PLUS

struct clksel_rate {
	u32			val;
	u8			div;
	u8			flags;
};

struct clksel {
	struct clk		 *parent;
	const struct clksel_rate *rates;
};

struct dpll_data {
	void __iomem		*mult_div1_reg;
	u32			mult_mask;
	u32			div1_mask;
	struct clk		*clk_bypass;
	struct clk		*clk_ref;
	void __iomem		*control_reg;
	u32			enable_mask;
	unsigned int		rate_tolerance;
	unsigned long		last_rounded_rate;
	u16			last_rounded_m;
	u16			max_multiplier;
	u8			last_rounded_n;
	u8			min_divider;
	u8			max_divider;
	u8			modes;
#if defined(CONFIG_ARCH_OMAP3) || defined(CONFIG_ARCH_OMAP4)
	void __iomem		*autoidle_reg;
	void __iomem		*idlest_reg;
	u32			autoidle_mask;
	u32			freqsel_mask;
	u32			idlest_mask;
	u8			auto_recal_bit;
	u8			recal_en_bit;
	u8			recal_st_bit;
	u8			flags;
#  endif
};

#endif

struct clk {
	struct list_head	node;
	const struct clkops	*ops;
	const char		*name;
	struct clk		*parent;
	struct list_head	children;
	struct list_head	sibling;	/* node for children */
	unsigned long		rate;
	void __iomem		*enable_reg;
	unsigned long		(*recalc)(struct clk *);
	int			(*set_rate)(struct clk *, unsigned long);
	long			(*round_rate)(struct clk *, unsigned long);
	void			(*init)(struct clk *);
	__u8			enable_bit;
	__s8			usecount;
	u8			fixed_div;
	u8			flags;
#ifdef CONFIG_ARCH_OMAP2PLUS
	void __iomem		*clksel_reg;
	u32			clksel_mask;
	const struct clksel	*clksel;
	struct dpll_data	*dpll_data;
	const char		*clkdm_name;
	struct clockdomain	*clkdm;
#else
	__u8			rate_offset;
	__u8			src_offset;
#endif
#if defined(CONFIG_PM_DEBUG) && defined(CONFIG_DEBUG_FS)
	struct dentry		*dent;	/* For visible tree hierarchy */
#endif
};

struct cpufreq_frequency_table;

struct clk_functions {
	int		(*clk_enable)(struct clk *clk);
	void		(*clk_disable)(struct clk *clk);
	long		(*clk_round_rate)(struct clk *clk, unsigned long rate);
	int		(*clk_set_rate)(struct clk *clk, unsigned long rate);
	int		(*clk_set_parent)(struct clk *clk, struct clk *parent);
	void		(*clk_allow_idle)(struct clk *clk);
	void		(*clk_deny_idle)(struct clk *clk);
	void		(*clk_disable_unused)(struct clk *clk);
#ifdef CONFIG_CPU_FREQ
	void		(*clk_init_cpufreq_table)(struct cpufreq_frequency_table **);
	void		(*clk_exit_cpufreq_table)(struct cpufreq_frequency_table **);
#endif
};

extern int mpurate;

extern int clk_init(struct clk_functions *custom_clocks);
extern void clk_preinit(struct clk *clk);
extern int clk_register(struct clk *clk);
extern void clk_reparent(struct clk *child, struct clk *parent);
extern void clk_unregister(struct clk *clk);
extern void propagate_rate(struct clk *clk);
extern void recalculate_root_clocks(void);
extern unsigned long followparent_recalc(struct clk *clk);
extern void clk_enable_init_clocks(void);
unsigned long omap_fixed_divisor_recalc(struct clk *clk);
#ifdef CONFIG_CPU_FREQ
extern void clk_init_cpufreq_table(struct cpufreq_frequency_table **table);
extern void clk_exit_cpufreq_table(struct cpufreq_frequency_table **table);
#endif
extern struct clk *omap_clk_get_by_name(const char *name);

extern const struct clkops clkops_null;

extern struct clk dummy_ck;

struct clocksource *get_clocksource_32k(void);

/* Clock flags */
#define ENABLE_REG_32BIT	(1 << 0)	/* Use 32-bit access */
#define CLOCK_IDLE_CONTROL	(1 << 1)
#define CLOCK_NO_IDLE_PARENT	(1 << 2)
#define ENABLE_ON_INIT		(1 << 3)	/* Enable upon framework init */
#define INVERT_ENABLE		(1 << 4)	/* 0 enables, 1 disables */

/* Clksel_rate flags */
#define RATE_IN_242X		(1 << 0)
#define RATE_IN_243X		(1 << 1)
#define RATE_IN_3XXX		(1 << 2)	/* rates common to all OMAP3 */
#define RATE_IN_3430ES2		(1 << 3)	/* 3430ES2 rates only */
#define RATE_IN_36XX		(1 << 4)
#define RATE_IN_4430		(1 << 5)

#define RATE_IN_24XX		(RATE_IN_242X | RATE_IN_243X)

#define RATE_IN_3430ES2PLUS	(RATE_IN_3430ES2 | RATE_IN_36XX)

#endif
