#ifndef __ASM_ARM_SMP_H
#define __ASM_ARM_SMP_H

#include <linux/threads.h>
#include <linux/cpumask.h>
#include <linux/thread_info.h>

#include <asm/arch/smp.h>

#ifndef CONFIG_SMP
# error "<asm-arm/smp.h> included in non-SMP build"
#endif

#define raw_smp_processor_id() (current_thread_info()->cpu)

#define PROC_CHANGE_PENALTY		15

struct seq_file;

extern void show_ipi_list(struct seq_file *p);

asmlinkage void do_IPI(struct pt_regs *regs);

extern void smp_init_cpus(void);

extern void smp_store_cpu_info(unsigned int cpuid);

extern void smp_cross_call(cpumask_t callmap);

extern void smp_send_timer(void);

extern int boot_secondary(unsigned int cpu, struct task_struct *);

asmlinkage void secondary_start_kernel(void);

extern void platform_secondary_init(unsigned int cpu);

struct secondary_data {
	unsigned long pgdir;
	void *stack;
};
extern struct secondary_data secondary_data;

extern int __cpu_disable(void);
extern int mach_cpu_disable(unsigned int cpu);

extern void __cpu_die(unsigned int cpu);
extern void cpu_die(void);

extern void platform_cpu_die(unsigned int cpu);
extern int platform_cpu_kill(unsigned int cpu);
extern void platform_cpu_enable(unsigned int cpu);

#ifdef CONFIG_LOCAL_TIMERS
extern void local_timer_setup(unsigned int cpu);

extern void local_timer_stop(unsigned int cpu);

extern int local_timer_ack(void);

#else

static inline void local_timer_setup(unsigned int cpu)
{
}

static inline void local_timer_stop(unsigned int cpu)
{
}

#endif

extern void show_local_irqs(struct seq_file *);

asmlinkage void do_local_timer(struct pt_regs *);

#endif /* ifndef __ASM_ARM_SMP_H */
