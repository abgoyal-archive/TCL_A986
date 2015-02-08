#ifndef __LINUX_SMP_H
#define __LINUX_SMP_H



extern void cpu_idle(void);

#ifdef CONFIG_SMP

#include <linux/preempt.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/thread_info.h>
#include <asm/smp.h>


extern void smp_send_stop(void);

extern void smp_send_reschedule(int cpu);


extern void smp_prepare_cpus(unsigned int max_cpus);

extern int __cpu_up(unsigned int cpunum);

extern void smp_cpus_done(unsigned int max_cpus);

int smp_call_function(void(*func)(void *info), void *info, int retry, int wait);

int on_each_cpu(void (*func) (void *info), void *info, int retry, int wait);

#define MSG_ALL_BUT_SELF	0x8000	/* Assume <32768 CPU's */
#define MSG_ALL			0x8001

#define MSG_INVALIDATE_TLB	0x0001	/* Remote processor TLB invalidate */
#define MSG_STOP_CPU		0x0002	/* Sent to shut down slave CPU's
					 * when rebooting
					 */
#define MSG_RESCHEDULE		0x0003	/* Reschedule request from master CPU*/
#define MSG_CALL_FUNCTION       0x0004  /* Call function on all other CPUs */

void smp_prepare_boot_cpu(void);

#else /* !SMP */

#define raw_smp_processor_id()			0
#define hard_smp_processor_id()			0
static inline int up_smp_call_function(void)
{
	return 0;
}
#define smp_call_function(func,info,retry,wait)	(up_smp_call_function())
#define on_each_cpu(func,info,retry,wait)	\
	({					\
		local_irq_disable();		\
		func(info);			\
		local_irq_enable();		\
		0;				\
	})
static inline void smp_send_reschedule(int cpu) { }
#define num_booting_cpus()			1
#define smp_prepare_boot_cpu()			do {} while (0)

#endif /* !SMP */

#ifdef CONFIG_DEBUG_PREEMPT
  extern unsigned int debug_smp_processor_id(void);
# define smp_processor_id() debug_smp_processor_id()
#else
# define smp_processor_id() raw_smp_processor_id()
#endif

#define get_cpu()		({ preempt_disable(); smp_processor_id(); })
#define put_cpu()		preempt_enable()
#define put_cpu_no_resched()	preempt_enable_no_resched()

void smp_setup_processor_id(void);

#endif /* __LINUX_SMP_H */
