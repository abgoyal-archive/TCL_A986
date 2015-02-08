

#include <linux/cpufreq.h>

struct s3c_cpufreq_info;
struct s3c_cpufreq_board;
struct s3c_iotimings;

struct s3c_freq {
	unsigned long	fclk;
	unsigned long	armclk;
	unsigned long	hclk_tns;	/* in 10ths of ns */
	unsigned long	hclk;
	unsigned long	pclk;
};

struct s3c_cpufreq_freqs {
	struct cpufreq_freqs	freqs;
	struct s3c_freq		old;
	struct s3c_freq		new;

	unsigned int		pll_changing:1;
};

#define to_s3c_cpufreq(_cf) container_of(_cf, struct s3c_cpufreq_freqs, freqs)

struct s3c_clkdivs {
	int		p_divisor;
	int		h_divisor;
	int		arm_divisor;
	unsigned char	dvs;
};

#define PLLVAL(_m, _p, _s) (((_m) << 12) | ((_p) << 4) | (_s))

struct s3c_pllval {
	unsigned long		freq;
	unsigned long		pll_reg;
};

struct s3c_cpufreq_board {
	unsigned int	refresh;
	unsigned int	auto_io:1;	/* automatically init io timings. */
	unsigned int	need_io:1;	/* set if needs io timing support. */

	/* any non-zero field in here is taken as an upper limit. */
	struct s3c_freq	max;	/* frequency limits */
};

/* Things depending on frequency scaling. */
#ifdef CONFIG_CPU_FREQ_S3C
#define __init_or_cpufreq
#else
#define __init_or_cpufreq __init
#endif

/* Board functions */

#ifdef CONFIG_CPU_FREQ_S3C
extern int s3c_cpufreq_setboard(struct s3c_cpufreq_board *board);
#else

static inline int s3c_cpufreq_setboard(struct s3c_cpufreq_board *board)
{
	return 0;
}
#endif  /* CONFIG_CPU_FREQ_S3C */
