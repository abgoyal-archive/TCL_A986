
#ifndef __ASM_ARM_CPU_H
#define __ASM_ARM_CPU_H

#include <linux/percpu.h>
#include <linux/cpu.h>

struct cpuinfo_arm {
	struct cpu	cpu;
#ifdef CONFIG_SMP
	struct task_struct *idle;
	unsigned int	loops_per_jiffy;
#endif
};

DECLARE_PER_CPU(struct cpuinfo_arm, cpu_data);

#endif
