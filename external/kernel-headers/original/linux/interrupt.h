/* interrupt.h */
#ifndef _LINUX_INTERRUPT_H
#define _LINUX_INTERRUPT_H

#include <linux/kernel.h>
#include <linux/linkage.h>
#include <linux/bitops.h>
#include <linux/preempt.h>
#include <linux/cpumask.h>
#include <linux/irqreturn.h>
#include <linux/hardirq.h>
#include <linux/sched.h>
#include <linux/irqflags.h>
#include <asm/atomic.h>
#include <asm/ptrace.h>
#include <asm/system.h>

#define IRQF_TRIGGER_NONE	0x00000000
#define IRQF_TRIGGER_RISING	0x00000001
#define IRQF_TRIGGER_FALLING	0x00000002
#define IRQF_TRIGGER_HIGH	0x00000004
#define IRQF_TRIGGER_LOW	0x00000008
#define IRQF_TRIGGER_MASK	(IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW | \
				 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)
#define IRQF_TRIGGER_PROBE	0x00000010

#define IRQF_DISABLED		0x00000020
#define IRQF_SAMPLE_RANDOM	0x00000040
#define IRQF_SHARED		0x00000080
#define IRQF_PROBE_SHARED	0x00000100
#define IRQF_TIMER		0x00000200
#define IRQF_PERCPU		0x00000400

#define SA_INTERRUPT		IRQF_DISABLED
#define SA_SAMPLE_RANDOM	IRQF_SAMPLE_RANDOM
#define SA_SHIRQ		IRQF_SHARED
#define SA_PROBEIRQ		IRQF_PROBE_SHARED
#define SA_PERCPU		IRQF_PERCPU

#define SA_TRIGGER_LOW		IRQF_TRIGGER_LOW
#define SA_TRIGGER_HIGH		IRQF_TRIGGER_HIGH
#define SA_TRIGGER_FALLING	IRQF_TRIGGER_FALLING
#define SA_TRIGGER_RISING	IRQF_TRIGGER_RISING
#define SA_TRIGGER_MASK		IRQF_TRIGGER_MASK

struct irqaction {
	irqreturn_t (*handler)(int, void *, struct pt_regs *);
	unsigned long flags;
	cpumask_t mask;
	const char *name;
	void *dev_id;
	struct irqaction *next;
	int irq;
	struct proc_dir_entry *dir;
};

extern irqreturn_t no_action(int cpl, void *dev_id, struct pt_regs *regs);
extern int request_irq(unsigned int,
		       irqreturn_t (*handler)(int, void *, struct pt_regs *),
		       unsigned long, const char *, void *);
extern void free_irq(unsigned int, void *);

#ifdef CONFIG_LOCKDEP
# define local_irq_enable_in_hardirq()	do { } while (0)
#else
# define local_irq_enable_in_hardirq()	local_irq_enable()
#endif

#ifdef CONFIG_GENERIC_HARDIRQS
extern void disable_irq_nosync(unsigned int irq);
extern void disable_irq(unsigned int irq);
extern void enable_irq(unsigned int irq);

static inline void disable_irq_nosync_lockdep(unsigned int irq)
{
	disable_irq_nosync(irq);
#ifdef CONFIG_LOCKDEP
	local_irq_disable();
#endif
}

static inline void disable_irq_lockdep(unsigned int irq)
{
	disable_irq(irq);
#ifdef CONFIG_LOCKDEP
	local_irq_disable();
#endif
}

static inline void enable_irq_lockdep(unsigned int irq)
{
#ifdef CONFIG_LOCKDEP
	local_irq_enable();
#endif
	enable_irq(irq);
}

/* IRQ wakeup (PM) control: */
extern int set_irq_wake(unsigned int irq, unsigned int on);

static inline int enable_irq_wake(unsigned int irq)
{
	return set_irq_wake(irq, 1);
}

static inline int disable_irq_wake(unsigned int irq)
{
	return set_irq_wake(irq, 0);
}

#else /* !CONFIG_GENERIC_HARDIRQS */
# ifndef CONFIG_LOCKDEP
#  define disable_irq_nosync_lockdep(irq)	disable_irq_nosync(irq)
#  define disable_irq_lockdep(irq)		disable_irq(irq)
#  define enable_irq_lockdep(irq)		enable_irq(irq)
# endif

#endif /* CONFIG_GENERIC_HARDIRQS */

#ifndef __ARCH_SET_SOFTIRQ_PENDING
#define set_softirq_pending(x) (local_softirq_pending() = (x))
#define or_softirq_pending(x)  (local_softirq_pending() |= (x))
#endif

#ifndef CONFIG_SMP
static inline void __deprecated cli(void)
{
	local_irq_disable();
}
static inline void __deprecated sti(void)
{
	local_irq_enable();
}
static inline void __deprecated save_flags(unsigned long *x)
{
	local_save_flags(*x);
}
#define save_flags(x) save_flags(&x)
static inline void __deprecated restore_flags(unsigned long x)
{
	local_irq_restore(x);
}

static inline void __deprecated save_and_cli(unsigned long *x)
{
	local_irq_save(*x);
}
#define save_and_cli(x)	save_and_cli(&x)
#endif /* CONFIG_SMP */

extern void local_bh_disable(void);
extern void __local_bh_enable(void);
extern void _local_bh_enable(void);
extern void local_bh_enable(void);
extern void local_bh_enable_ip(unsigned long ip);


enum
{
	HI_SOFTIRQ=0,
	TIMER_SOFTIRQ,
	NET_TX_SOFTIRQ,
	NET_RX_SOFTIRQ,
	BLOCK_SOFTIRQ,
	TASKLET_SOFTIRQ
};


struct softirq_action
{
	void	(*action)(struct softirq_action *);
	void	*data;
};

asmlinkage void do_softirq(void);
extern void open_softirq(int nr, void (*action)(struct softirq_action*), void *data);
extern void softirq_init(void);
#define __raise_softirq_irqoff(nr) do { or_softirq_pending(1UL << (nr)); } while (0)
extern void FASTCALL(raise_softirq_irqoff(unsigned int nr));
extern void FASTCALL(raise_softirq(unsigned int nr));



struct tasklet_struct
{
	struct tasklet_struct *next;
	unsigned long state;
	atomic_t count;
	void (*func)(unsigned long);
	unsigned long data;
};

#define DECLARE_TASKLET(name, func, data) \
struct tasklet_struct name = { NULL, 0, ATOMIC_INIT(0), func, data }

#define DECLARE_TASKLET_DISABLED(name, func, data) \
struct tasklet_struct name = { NULL, 0, ATOMIC_INIT(1), func, data }


enum
{
	TASKLET_STATE_SCHED,	/* Tasklet is scheduled for execution */
	TASKLET_STATE_RUN	/* Tasklet is running (SMP only) */
};

#ifdef CONFIG_SMP
static inline int tasklet_trylock(struct tasklet_struct *t)
{
	return !test_and_set_bit(TASKLET_STATE_RUN, &(t)->state);
}

static inline void tasklet_unlock(struct tasklet_struct *t)
{
	smp_mb__before_clear_bit(); 
	clear_bit(TASKLET_STATE_RUN, &(t)->state);
}

static inline void tasklet_unlock_wait(struct tasklet_struct *t)
{
	while (test_bit(TASKLET_STATE_RUN, &(t)->state)) { barrier(); }
}
#else
#define tasklet_trylock(t) 1
#define tasklet_unlock_wait(t) do { } while (0)
#define tasklet_unlock(t) do { } while (0)
#endif

extern void FASTCALL(__tasklet_schedule(struct tasklet_struct *t));

static inline void tasklet_schedule(struct tasklet_struct *t)
{
	if (!test_and_set_bit(TASKLET_STATE_SCHED, &t->state))
		__tasklet_schedule(t);
}

extern void FASTCALL(__tasklet_hi_schedule(struct tasklet_struct *t));

static inline void tasklet_hi_schedule(struct tasklet_struct *t)
{
	if (!test_and_set_bit(TASKLET_STATE_SCHED, &t->state))
		__tasklet_hi_schedule(t);
}


static inline void tasklet_disable_nosync(struct tasklet_struct *t)
{
	atomic_inc(&t->count);
	smp_mb__after_atomic_inc();
}

static inline void tasklet_disable(struct tasklet_struct *t)
{
	tasklet_disable_nosync(t);
	tasklet_unlock_wait(t);
	smp_mb();
}

static inline void tasklet_enable(struct tasklet_struct *t)
{
	smp_mb__before_atomic_dec();
	atomic_dec(&t->count);
}

static inline void tasklet_hi_enable(struct tasklet_struct *t)
{
	smp_mb__before_atomic_dec();
	atomic_dec(&t->count);
}

extern void tasklet_kill(struct tasklet_struct *t);
extern void tasklet_kill_immediate(struct tasklet_struct *t, unsigned int cpu);
extern void tasklet_init(struct tasklet_struct *t,
			 void (*func)(unsigned long), unsigned long data);


#if defined(CONFIG_GENERIC_HARDIRQS) && !defined(CONFIG_GENERIC_IRQ_PROBE) 
static inline unsigned long probe_irq_on(void)
{
	return 0;
}
static inline int probe_irq_off(unsigned long val)
{
	return 0;
}
static inline unsigned int probe_irq_mask(unsigned long val)
{
	return 0;
}
#else
extern unsigned long probe_irq_on(void);	/* returns 0 on failure */
extern int probe_irq_off(unsigned long);	/* returns 0 or negative on failure */
extern unsigned int probe_irq_mask(unsigned long);	/* returns mask of ISA interrupts */
#endif

#endif
