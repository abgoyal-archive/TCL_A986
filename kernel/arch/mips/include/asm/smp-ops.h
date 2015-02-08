
#ifndef __ASM_SMP_OPS_H
#define __ASM_SMP_OPS_H

#ifdef CONFIG_SMP

#include <linux/cpumask.h>

struct task_struct;

struct plat_smp_ops {
	void (*send_ipi_single)(int cpu, unsigned int action);
	void (*send_ipi_mask)(const struct cpumask *mask, unsigned int action);
	void (*init_secondary)(void);
	void (*smp_finish)(void);
	void (*cpus_done)(void);
	void (*boot_secondary)(int cpu, struct task_struct *idle);
	void (*smp_setup)(void);
	void (*prepare_cpus)(unsigned int max_cpus);
#ifdef CONFIG_HOTPLUG_CPU
	int (*cpu_disable)(void);
	void (*cpu_die)(unsigned int cpu);
#endif
};

extern void register_smp_ops(struct plat_smp_ops *ops);

static inline void plat_smp_setup(void)
{
	extern struct plat_smp_ops *mp_ops;	/* private */

	mp_ops->smp_setup();
}

#else /* !CONFIG_SMP */

struct plat_smp_ops;

static inline void plat_smp_setup(void)
{
	/* UP, nothing to do ...  */
}

static inline void register_smp_ops(struct plat_smp_ops *ops)
{
}

#endif /* !CONFIG_SMP */

extern struct plat_smp_ops up_smp_ops;
extern struct plat_smp_ops cmp_smp_ops;
extern struct plat_smp_ops vsmp_smp_ops;

#endif /* __ASM_SMP_OPS_H */
