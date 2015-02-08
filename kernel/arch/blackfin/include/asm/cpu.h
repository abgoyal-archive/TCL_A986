

#ifndef __ASM_BLACKFIN_CPU_H
#define __ASM_BLACKFIN_CPU_H

#include <linux/percpu.h>

struct task_struct;

struct blackfin_cpudata {
	struct cpu cpu;
	struct task_struct *idle;
	unsigned int imemctl;
	unsigned int dmemctl;
};

DECLARE_PER_CPU(struct blackfin_cpudata, cpu_data);

#endif
