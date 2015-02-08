
#ifndef _ASM_POWERPC_PERCPU_H_
#define _ASM_POWERPC_PERCPU_H_
#ifdef __powerpc64__
#include <linux/compiler.h>


#ifdef CONFIG_SMP

#include <asm/paca.h>

#define __per_cpu_offset(cpu) (paca[cpu].data_offset)
#define __my_cpu_offset local_paca->data_offset
#define per_cpu_offset(x) (__per_cpu_offset(x))

#endif /* CONFIG_SMP */
#endif /* __powerpc64__ */

#include <asm-generic/percpu.h>

#endif /* _ASM_POWERPC_PERCPU_H_ */
