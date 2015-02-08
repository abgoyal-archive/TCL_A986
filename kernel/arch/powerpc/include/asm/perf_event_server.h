

#include <linux/types.h>
#include <asm/hw_irq.h>

#define MAX_HWEVENTS		8
#define MAX_EVENT_ALTERNATIVES	8
#define MAX_LIMITED_HWCOUNTERS	2

struct power_pmu {
	const char	*name;
	int		n_counter;
	int		max_alternatives;
	unsigned long	add_fields;
	unsigned long	test_adder;
	int		(*compute_mmcr)(u64 events[], int n_ev,
				unsigned int hwc[], unsigned long mmcr[]);
	int		(*get_constraint)(u64 event_id, unsigned long *mskp,
				unsigned long *valp);
	int		(*get_alternatives)(u64 event_id, unsigned int flags,
				u64 alt[]);
	void		(*disable_pmc)(unsigned int pmc, unsigned long mmcr[]);
	int		(*limited_pmc_event)(u64 event_id);
	u32		flags;
	int		n_generic;
	int		*generic_events;
	int		(*cache_events)[PERF_COUNT_HW_CACHE_MAX]
			       [PERF_COUNT_HW_CACHE_OP_MAX]
			       [PERF_COUNT_HW_CACHE_RESULT_MAX];
};

#define PPMU_LIMITED_PMC5_6	1	/* PMC5/6 have limited function */
#define PPMU_ALT_SIPR		2	/* uses alternate posn for SIPR/HV */

#define PPMU_LIMITED_PMC_OK	1	/* can put this on a limited PMC */
#define PPMU_LIMITED_PMC_REQD	2	/* have to put this on a limited PMC */
#define PPMU_ONLY_COUNT_RUN	4	/* only counting in run state */

extern int register_power_pmu(struct power_pmu *);

struct pt_regs;
extern unsigned long perf_misc_flags(struct pt_regs *regs);
extern unsigned long perf_instruction_pointer(struct pt_regs *regs);

#define PERF_EVENT_INDEX_OFFSET	1

#ifdef CONFIG_PPC_PERF_CTRS
#define perf_misc_flags(regs)	perf_misc_flags(regs)
#endif

