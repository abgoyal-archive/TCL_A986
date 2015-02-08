

#ifndef __ARM_PMU_H__
#define __ARM_PMU_H__

enum arm_pmu_type {
	ARM_PMU_DEVICE_CPU	= 0,
	ARM_NUM_PMU_DEVICES,
};

#ifdef CONFIG_CPU_HAS_PMU

extern struct platform_device *
reserve_pmu(enum arm_pmu_type device);

extern int
release_pmu(struct platform_device *pdev);

extern int
init_pmu(enum arm_pmu_type device);

#else /* CONFIG_CPU_HAS_PMU */

#include <linux/err.h>

static inline struct platform_device *
reserve_pmu(enum arm_pmu_type device)
{
	return ERR_PTR(-ENODEV);
}

static inline int
release_pmu(struct platform_device *pdev)
{
	return -ENODEV;
}

static inline int
init_pmu(enum arm_pmu_type device)
{
	return -ENODEV;
}

#endif /* CONFIG_CPU_HAS_PMU */

#endif /* __ARM_PMU_H__ */
