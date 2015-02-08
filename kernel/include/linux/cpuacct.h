

#ifndef _CPUACCT_H_
#define _CPUACCT_H_

#include <linux/cgroup.h>

#ifdef CONFIG_CGROUP_CPUACCT

struct cpuacct_charge_calls {
	/*
	 * Platforms can take advantage of this data and use
	 * per-cpu allocations if necessary.
	 */
	void (*init) (void **cpuacct_data);
	void (*charge) (void *cpuacct_data,  u64 cputime, unsigned int cpu);
	void (*cpufreq_show) (void *cpuacct_data, struct cgroup_map_cb *cb);
	/* Returns power consumed in milliWatt seconds */
	u64 (*power_usage) (void *cpuacct_data);
};

int cpuacct_charge_register(struct cpuacct_charge_calls *fn);

#endif /* CONFIG_CGROUP_CPUACCT */

#endif // _CPUACCT_H_
