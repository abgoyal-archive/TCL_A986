
#include <linux/moduleparam.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/notifier.h>
#include <linux/rcupdate.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/cpu.h>

/* Global control variables for rcupdate callback mechanism. */
struct rcu_ctrlblk {
	struct rcu_head *rcucblist;	/* List of pending callbacks (CBs). */
	struct rcu_head **donetail;	/* ->next pointer of last "done" CB. */
	struct rcu_head **curtail;	/* ->next pointer of last CB. */
};

/* Definition for rcupdate control block. */
static struct rcu_ctrlblk rcu_sched_ctrlblk = {
	.donetail	= &rcu_sched_ctrlblk.rcucblist,
	.curtail	= &rcu_sched_ctrlblk.rcucblist,
};

static struct rcu_ctrlblk rcu_bh_ctrlblk = {
	.donetail	= &rcu_bh_ctrlblk.rcucblist,
	.curtail	= &rcu_bh_ctrlblk.rcucblist,
};

#ifdef CONFIG_DEBUG_LOCK_ALLOC
int rcu_scheduler_active __read_mostly;
EXPORT_SYMBOL_GPL(rcu_scheduler_active);
#endif /* #ifdef CONFIG_DEBUG_LOCK_ALLOC */

#ifdef CONFIG_NO_HZ

static long rcu_dynticks_nesting = 1;

void rcu_enter_nohz(void)
{
	if (--rcu_dynticks_nesting == 0)
		rcu_sched_qs(0); /* implies rcu_bh_qsctr_inc(0) */
}

void rcu_exit_nohz(void)
{
	rcu_dynticks_nesting++;
}

#endif /* #ifdef CONFIG_NO_HZ */

static int rcu_qsctr_help(struct rcu_ctrlblk *rcp)
{
	unsigned long flags;

	local_irq_save(flags);
	if (rcp->rcucblist != NULL &&
	    rcp->donetail != rcp->curtail) {
		rcp->donetail = rcp->curtail;
		local_irq_restore(flags);
		return 1;
	}
	local_irq_restore(flags);

	return 0;
}

void rcu_sched_qs(int cpu)
{
	if (rcu_qsctr_help(&rcu_sched_ctrlblk) +
	    rcu_qsctr_help(&rcu_bh_ctrlblk))
		raise_softirq(RCU_SOFTIRQ);
}

void rcu_bh_qs(int cpu)
{
	if (rcu_qsctr_help(&rcu_bh_ctrlblk))
		raise_softirq(RCU_SOFTIRQ);
}

void rcu_check_callbacks(int cpu, int user)
{
	if (user ||
	    (idle_cpu(cpu) &&
	     !in_softirq() &&
	     hardirq_count() <= (1 << HARDIRQ_SHIFT)))
		rcu_sched_qs(cpu);
	else if (!in_softirq())
		rcu_bh_qs(cpu);
}

static void __rcu_process_callbacks(struct rcu_ctrlblk *rcp)
{
	struct rcu_head *next, *list;
	unsigned long flags;

	/* If no RCU callbacks ready to invoke, just return. */
	if (&rcp->rcucblist == rcp->donetail)
		return;

	/* Move the ready-to-invoke callbacks to a local list. */
	local_irq_save(flags);
	list = rcp->rcucblist;
	rcp->rcucblist = *rcp->donetail;
	*rcp->donetail = NULL;
	if (rcp->curtail == rcp->donetail)
		rcp->curtail = &rcp->rcucblist;
	rcp->donetail = &rcp->rcucblist;
	local_irq_restore(flags);

	/* Invoke the callbacks on the local list. */
	while (list) {
		next = list->next;
		prefetch(next);
		list->func(list);
		list = next;
	}
}

static void rcu_process_callbacks(struct softirq_action *unused)
{
	__rcu_process_callbacks(&rcu_sched_ctrlblk);
	__rcu_process_callbacks(&rcu_bh_ctrlblk);
}

void synchronize_sched(void)
{
	cond_resched();
}
EXPORT_SYMBOL_GPL(synchronize_sched);

static void __call_rcu(struct rcu_head *head,
		       void (*func)(struct rcu_head *rcu),
		       struct rcu_ctrlblk *rcp)
{
	unsigned long flags;

	head->func = func;
	head->next = NULL;

	local_irq_save(flags);
	*rcp->curtail = head;
	rcp->curtail = &head->next;
	local_irq_restore(flags);
}

void call_rcu(struct rcu_head *head, void (*func)(struct rcu_head *rcu))
{
	__call_rcu(head, func, &rcu_sched_ctrlblk);
}
EXPORT_SYMBOL_GPL(call_rcu);

void call_rcu_bh(struct rcu_head *head, void (*func)(struct rcu_head *rcu))
{
	__call_rcu(head, func, &rcu_bh_ctrlblk);
}
EXPORT_SYMBOL_GPL(call_rcu_bh);

void rcu_barrier(void)
{
	struct rcu_synchronize rcu;

	init_rcu_head_on_stack(&rcu.head);
	init_completion(&rcu.completion);
	/* Will wake me after RCU finished. */
	call_rcu(&rcu.head, wakeme_after_rcu);
	/* Wait for it. */
	wait_for_completion(&rcu.completion);
	destroy_rcu_head_on_stack(&rcu.head);
}
EXPORT_SYMBOL_GPL(rcu_barrier);

void rcu_barrier_bh(void)
{
	struct rcu_synchronize rcu;

	init_rcu_head_on_stack(&rcu.head);
	init_completion(&rcu.completion);
	/* Will wake me after RCU finished. */
	call_rcu_bh(&rcu.head, wakeme_after_rcu);
	/* Wait for it. */
	wait_for_completion(&rcu.completion);
	destroy_rcu_head_on_stack(&rcu.head);
}
EXPORT_SYMBOL_GPL(rcu_barrier_bh);

void rcu_barrier_sched(void)
{
	struct rcu_synchronize rcu;

	init_rcu_head_on_stack(&rcu.head);
	init_completion(&rcu.completion);
	/* Will wake me after RCU finished. */
	call_rcu_sched(&rcu.head, wakeme_after_rcu);
	/* Wait for it. */
	wait_for_completion(&rcu.completion);
	destroy_rcu_head_on_stack(&rcu.head);
}
EXPORT_SYMBOL_GPL(rcu_barrier_sched);

void __init rcu_init(void)
{
	open_softirq(RCU_SOFTIRQ, rcu_process_callbacks);
}

#include "rcutiny_plugin.h"
