
#ifndef LINUX_HARDIRQ_H
#define LINUX_HARDIRQ_H

#include <linux/preempt.h>
#ifdef CONFIG_PREEMPT
#include <linux/smp_lock.h>
#endif
#include <linux/lockdep.h>
#include <linux/ftrace_irq.h>
#include <asm/hardirq.h>
#include <asm/system.h>

#define PREEMPT_BITS	8
#define SOFTIRQ_BITS	8
#define NMI_BITS	1

#define MAX_HARDIRQ_BITS 10

#ifndef HARDIRQ_BITS
# define HARDIRQ_BITS	MAX_HARDIRQ_BITS
#endif

#if HARDIRQ_BITS > MAX_HARDIRQ_BITS
#error HARDIRQ_BITS too high!
#endif

#define PREEMPT_SHIFT	0
#define SOFTIRQ_SHIFT	(PREEMPT_SHIFT + PREEMPT_BITS)
#define HARDIRQ_SHIFT	(SOFTIRQ_SHIFT + SOFTIRQ_BITS)
#define NMI_SHIFT	(HARDIRQ_SHIFT + HARDIRQ_BITS)

#define __IRQ_MASK(x)	((1UL << (x))-1)

#define PREEMPT_MASK	(__IRQ_MASK(PREEMPT_BITS) << PREEMPT_SHIFT)
#define SOFTIRQ_MASK	(__IRQ_MASK(SOFTIRQ_BITS) << SOFTIRQ_SHIFT)
#define HARDIRQ_MASK	(__IRQ_MASK(HARDIRQ_BITS) << HARDIRQ_SHIFT)
#define NMI_MASK	(__IRQ_MASK(NMI_BITS)     << NMI_SHIFT)

#define PREEMPT_OFFSET	(1UL << PREEMPT_SHIFT)
#define SOFTIRQ_OFFSET	(1UL << SOFTIRQ_SHIFT)
#define HARDIRQ_OFFSET	(1UL << HARDIRQ_SHIFT)
#define NMI_OFFSET	(1UL << NMI_SHIFT)

#ifndef PREEMPT_ACTIVE
#define PREEMPT_ACTIVE_BITS	1
#define PREEMPT_ACTIVE_SHIFT	(NMI_SHIFT + NMI_BITS)
#define PREEMPT_ACTIVE	(__IRQ_MASK(PREEMPT_ACTIVE_BITS) << PREEMPT_ACTIVE_SHIFT)
#endif

#if PREEMPT_ACTIVE < (1 << (NMI_SHIFT + NMI_BITS))
#error PREEMPT_ACTIVE is too low!
#endif

#define hardirq_count()	(preempt_count() & HARDIRQ_MASK)
#define softirq_count()	(preempt_count() & SOFTIRQ_MASK)
#define irq_count()	(preempt_count() & (HARDIRQ_MASK | SOFTIRQ_MASK \
				 | NMI_MASK))

#define in_irq()		(hardirq_count())
#define in_softirq()		(softirq_count())
#define in_interrupt()		(irq_count())

#define in_nmi()	(preempt_count() & NMI_MASK)

#if defined(CONFIG_PREEMPT)
# define PREEMPT_INATOMIC_BASE kernel_locked()
# define PREEMPT_CHECK_OFFSET 1
#else
# define PREEMPT_INATOMIC_BASE 0
# define PREEMPT_CHECK_OFFSET 0
#endif

#define in_atomic()	((preempt_count() & ~PREEMPT_ACTIVE) != PREEMPT_INATOMIC_BASE)

#define in_atomic_preempt_off() \
		((preempt_count() & ~PREEMPT_ACTIVE) != PREEMPT_CHECK_OFFSET)

#ifdef CONFIG_PREEMPT
# define preemptible()	(preempt_count() == 0 && !irqs_disabled())
# define IRQ_EXIT_OFFSET (HARDIRQ_OFFSET-1)
#else
# define preemptible()	0
# define IRQ_EXIT_OFFSET HARDIRQ_OFFSET
#endif

#if defined(CONFIG_SMP) || defined(CONFIG_GENERIC_HARDIRQS)
extern void synchronize_irq(unsigned int irq);
#else
# define synchronize_irq(irq)	barrier()
#endif

struct task_struct;

#ifndef CONFIG_VIRT_CPU_ACCOUNTING
static inline void account_system_vtime(struct task_struct *tsk)
{
}
#endif

#if defined(CONFIG_NO_HZ)
#if defined(CONFIG_TINY_RCU)
extern void rcu_enter_nohz(void);
extern void rcu_exit_nohz(void);

static inline void rcu_irq_enter(void)
{
	rcu_exit_nohz();
}

static inline void rcu_irq_exit(void)
{
	rcu_enter_nohz();
}

static inline void rcu_nmi_enter(void)
{
}

static inline void rcu_nmi_exit(void)
{
}

#else
extern void rcu_irq_enter(void);
extern void rcu_irq_exit(void);
extern void rcu_nmi_enter(void);
extern void rcu_nmi_exit(void);
#endif
#else
# define rcu_irq_enter() do { } while (0)
# define rcu_irq_exit() do { } while (0)
# define rcu_nmi_enter() do { } while (0)
# define rcu_nmi_exit() do { } while (0)
#endif /* #if defined(CONFIG_NO_HZ) */

#define __irq_enter()					\
	do {						\
		account_system_vtime(current);		\
		add_preempt_count(HARDIRQ_OFFSET);	\
		trace_hardirq_enter();			\
	} while (0)

extern void irq_enter(void);

#define __irq_exit()					\
	do {						\
		trace_hardirq_exit();			\
		account_system_vtime(current);		\
		sub_preempt_count(HARDIRQ_OFFSET);	\
	} while (0)

extern void irq_exit(void);

#define nmi_enter()						\
	do {							\
		ftrace_nmi_enter();				\
		BUG_ON(in_nmi());				\
		add_preempt_count(NMI_OFFSET + HARDIRQ_OFFSET);	\
		lockdep_off();					\
		rcu_nmi_enter();				\
		trace_hardirq_enter();				\
	} while (0)

#define nmi_exit()						\
	do {							\
		trace_hardirq_exit();				\
		rcu_nmi_exit();					\
		lockdep_on();					\
		BUG_ON(!in_nmi());				\
		sub_preempt_count(NMI_OFFSET + HARDIRQ_OFFSET);	\
		ftrace_nmi_exit();				\
	} while (0)

#endif /* LINUX_HARDIRQ_H */
