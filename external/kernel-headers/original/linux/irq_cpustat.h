#ifndef __irq_cpustat_h
#define __irq_cpustat_h




#ifndef __ARCH_IRQ_STAT
extern irq_cpustat_t irq_stat[];		/* defined in asm/hardirq.h */
#define __IRQ_STAT(cpu, member)	(irq_stat[cpu].member)
#endif

  /* arch independent irq_stat fields */
#define local_softirq_pending() \
	__IRQ_STAT(smp_processor_id(), __softirq_pending)

  /* arch dependent irq_stat fields */
#define nmi_count(cpu)		__IRQ_STAT((cpu), __nmi_count)	/* i386 */

#endif	/* __irq_cpustat_h */
