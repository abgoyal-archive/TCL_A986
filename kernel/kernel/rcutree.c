
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/smp.h>
#include <linux/rcupdate.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/nmi.h>
#include <asm/atomic.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/completion.h>
#include <linux/moduleparam.h>
#include <linux/percpu.h>
#include <linux/notifier.h>
#include <linux/cpu.h>
#include <linux/mutex.h>
#include <linux/time.h>
#include <linux/kernel_stat.h>
#include <trace/rcu.h>

#include "rcutree.h"

/* Data structures. */

static struct lock_class_key rcu_node_class[NUM_RCU_LVLS];

#define RCU_STATE_INITIALIZER(structname) { \
	.level = { &structname.node[0] }, \
	.levelcnt = { \
		NUM_RCU_LVL_0,  /* root of hierarchy. */ \
		NUM_RCU_LVL_1, \
		NUM_RCU_LVL_2, \
		NUM_RCU_LVL_3, \
		NUM_RCU_LVL_4, /* == MAX_RCU_LVLS */ \
	}, \
	.signaled = RCU_GP_IDLE, \
	.gpnum = -300, \
	.completed = -300, \
	.onofflock = __RAW_SPIN_LOCK_UNLOCKED(&structname.onofflock), \
	.orphan_cbs_list = NULL, \
	.orphan_cbs_tail = &structname.orphan_cbs_list, \
	.orphan_qlen = 0, \
	.fqslock = __RAW_SPIN_LOCK_UNLOCKED(&structname.fqslock), \
	.n_force_qs = 0, \
	.n_force_qs_ngp = 0, \
	.name = #structname, \
}

struct rcu_state rcu_sched_state = RCU_STATE_INITIALIZER(rcu_sched_state);
DEFINE_PER_CPU(struct rcu_data, rcu_sched_data);

struct rcu_state rcu_bh_state = RCU_STATE_INITIALIZER(rcu_bh_state);
DEFINE_PER_CPU(struct rcu_data, rcu_bh_data);

int rcu_scheduler_active __read_mostly;
EXPORT_SYMBOL_GPL(rcu_scheduler_active);

static int rcu_gp_in_progress(struct rcu_state *rsp)
{
	return ACCESS_ONCE(rsp->completed) != ACCESS_ONCE(rsp->gpnum);
}

void rcu_sched_qs(int cpu)
{
	struct rcu_data *rdp = &per_cpu(rcu_sched_data, cpu);

	rdp->passed_quiesc_completed = rdp->gpnum - 1;
	barrier();
	rdp->passed_quiesc = 1;
}

void rcu_bh_qs(int cpu)
{
	struct rcu_data *rdp = &per_cpu(rcu_bh_data, cpu);

	rdp->passed_quiesc_completed = rdp->gpnum - 1;
	barrier();
	rdp->passed_quiesc = 1;
}

void rcu_note_context_switch(int cpu)
{
	rcu_sched_qs(cpu);
	rcu_preempt_note_context_switch(cpu);
}

#ifdef CONFIG_NO_HZ
DEFINE_PER_CPU(struct rcu_dynticks, rcu_dynticks) = {
	.dynticks_nesting = 1,
	.dynticks = 1,
};
#endif /* #ifdef CONFIG_NO_HZ */

static int blimit = 10;		/* Maximum callbacks per softirq. */
static int qhimark = 10000;	/* If this many pending, ignore blimit. */
static int qlowmark = 100;	/* Once only this many pending, use blimit. */

module_param(blimit, int, 0);
module_param(qhimark, int, 0);
module_param(qlowmark, int, 0);

DEFINE_TRACE(rcu_tree_call_rcu);
DEFINE_TRACE(rcu_tree_call_rcu_bh);
DEFINE_TRACE(rcu_tree_callback);

static void force_quiescent_state(struct rcu_state *rsp, int relaxed);
static int rcu_pending(int cpu);

long rcu_batches_completed_sched(void)
{
	return rcu_sched_state.completed;
}
EXPORT_SYMBOL_GPL(rcu_batches_completed_sched);

long rcu_batches_completed_bh(void)
{
	return rcu_bh_state.completed;
}
EXPORT_SYMBOL_GPL(rcu_batches_completed_bh);

void rcu_bh_force_quiescent_state(void)
{
	force_quiescent_state(&rcu_bh_state, 0);
}
EXPORT_SYMBOL_GPL(rcu_bh_force_quiescent_state);

void rcu_sched_force_quiescent_state(void)
{
	force_quiescent_state(&rcu_sched_state, 0);
}
EXPORT_SYMBOL_GPL(rcu_sched_force_quiescent_state);

static int
cpu_has_callbacks_ready_to_invoke(struct rcu_data *rdp)
{
	return &rdp->nxtlist != rdp->nxttail[RCU_DONE_TAIL];
}

static int
cpu_needs_another_gp(struct rcu_state *rsp, struct rcu_data *rdp)
{
	return *rdp->nxttail[RCU_DONE_TAIL] && !rcu_gp_in_progress(rsp);
}

static struct rcu_node *rcu_get_root(struct rcu_state *rsp)
{
	return &rsp->node[0];
}

#ifdef CONFIG_SMP

static int rcu_implicit_offline_qs(struct rcu_data *rdp)
{
	/*
	 * If the CPU is offline, it is in a quiescent state.  We can
	 * trust its state not to change because interrupts are disabled.
	 */
	if (cpu_is_offline(rdp->cpu)) {
		rdp->offline_fqs++;
		return 1;
	}

	/* If preemptable RCU, no point in sending reschedule IPI. */
	if (rdp->preemptable)
		return 0;

	/* The CPU is online, so send it a reschedule IPI. */
	if (rdp->cpu != smp_processor_id())
		smp_send_reschedule(rdp->cpu);
	else
		set_need_resched();
	rdp->resched_ipi++;
	return 0;
}

#endif /* #ifdef CONFIG_SMP */

#ifdef CONFIG_NO_HZ

void rcu_enter_nohz(void)
{
	unsigned long flags;
	struct rcu_dynticks *rdtp;

	smp_mb(); /* CPUs seeing ++ must see prior RCU read-side crit sects */
	local_irq_save(flags);
	rdtp = &__get_cpu_var(rcu_dynticks);
	rdtp->dynticks++;
	rdtp->dynticks_nesting--;
	WARN_ON_ONCE(rdtp->dynticks & 0x1);
	local_irq_restore(flags);
}

void rcu_exit_nohz(void)
{
	unsigned long flags;
	struct rcu_dynticks *rdtp;

	local_irq_save(flags);
	rdtp = &__get_cpu_var(rcu_dynticks);
	rdtp->dynticks++;
	rdtp->dynticks_nesting++;
	WARN_ON_ONCE(!(rdtp->dynticks & 0x1));
	local_irq_restore(flags);
	smp_mb(); /* CPUs seeing ++ must see later RCU read-side crit sects */
}

void rcu_nmi_enter(void)
{
	struct rcu_dynticks *rdtp = &__get_cpu_var(rcu_dynticks);

	if (rdtp->dynticks & 0x1)
		return;
	rdtp->dynticks_nmi++;
	WARN_ON_ONCE(!(rdtp->dynticks_nmi & 0x1));
	smp_mb(); /* CPUs seeing ++ must see later RCU read-side crit sects */
}

void rcu_nmi_exit(void)
{
	struct rcu_dynticks *rdtp = &__get_cpu_var(rcu_dynticks);

	if (rdtp->dynticks & 0x1)
		return;
	smp_mb(); /* CPUs seeing ++ must see prior RCU read-side crit sects */
	rdtp->dynticks_nmi++;
	WARN_ON_ONCE(rdtp->dynticks_nmi & 0x1);
}

void rcu_irq_enter(void)
{
	struct rcu_dynticks *rdtp = &__get_cpu_var(rcu_dynticks);

	if (rdtp->dynticks_nesting++)
		return;
	rdtp->dynticks++;
	WARN_ON_ONCE(!(rdtp->dynticks & 0x1));
	smp_mb(); /* CPUs seeing ++ must see later RCU read-side crit sects */
}

void rcu_irq_exit(void)
{
	struct rcu_dynticks *rdtp = &__get_cpu_var(rcu_dynticks);

	if (--rdtp->dynticks_nesting)
		return;
	smp_mb(); /* CPUs seeing ++ must see prior RCU read-side crit sects */
	rdtp->dynticks++;
	WARN_ON_ONCE(rdtp->dynticks & 0x1);

	/* If the interrupt queued a callback, get out of dyntick mode. */
	if (__get_cpu_var(rcu_sched_data).nxtlist ||
	    __get_cpu_var(rcu_bh_data).nxtlist)
		set_need_resched();
}

#ifdef CONFIG_SMP

static int dyntick_save_progress_counter(struct rcu_data *rdp)
{
	int ret;
	int snap;
	int snap_nmi;

	snap = rdp->dynticks->dynticks;
	snap_nmi = rdp->dynticks->dynticks_nmi;
	smp_mb();	/* Order sampling of snap with end of grace period. */
	rdp->dynticks_snap = snap;
	rdp->dynticks_nmi_snap = snap_nmi;
	ret = ((snap & 0x1) == 0) && ((snap_nmi & 0x1) == 0);
	if (ret)
		rdp->dynticks_fqs++;
	return ret;
}

static int rcu_implicit_dynticks_qs(struct rcu_data *rdp)
{
	long curr;
	long curr_nmi;
	long snap;
	long snap_nmi;

	curr = rdp->dynticks->dynticks;
	snap = rdp->dynticks_snap;
	curr_nmi = rdp->dynticks->dynticks_nmi;
	snap_nmi = rdp->dynticks_nmi_snap;
	smp_mb(); /* force ordering with cpu entering/leaving dynticks. */

	/*
	 * If the CPU passed through or entered a dynticks idle phase with
	 * no active irq/NMI handlers, then we can safely pretend that the CPU
	 * already acknowledged the request to pass through a quiescent
	 * state.  Either way, that CPU cannot possibly be in an RCU
	 * read-side critical section that started before the beginning
	 * of the current RCU grace period.
	 */
	if ((curr != snap || (curr & 0x1) == 0) &&
	    (curr_nmi != snap_nmi || (curr_nmi & 0x1) == 0)) {
		rdp->dynticks_fqs++;
		return 1;
	}

	/* Go check for the CPU being offline. */
	return rcu_implicit_offline_qs(rdp);
}

#endif /* #ifdef CONFIG_SMP */

#else /* #ifdef CONFIG_NO_HZ */

#ifdef CONFIG_SMP

static int dyntick_save_progress_counter(struct rcu_data *rdp)
{
	return 0;
}

static int rcu_implicit_dynticks_qs(struct rcu_data *rdp)
{
	return rcu_implicit_offline_qs(rdp);
}

#endif /* #ifdef CONFIG_SMP */

#endif /* #else #ifdef CONFIG_NO_HZ */

#ifdef CONFIG_RCU_CPU_STALL_DETECTOR

int rcu_cpu_stall_panicking __read_mostly;

static void record_gp_stall_check_time(struct rcu_state *rsp)
{
	rsp->gp_start = jiffies;
	rsp->jiffies_stall = jiffies + RCU_SECONDS_TILL_STALL_CHECK;
}

static void print_other_cpu_stall(struct rcu_state *rsp)
{
	int cpu;
	long delta;
	unsigned long flags;
	struct rcu_node *rnp = rcu_get_root(rsp);

	/* Only let one CPU complain about others per time interval. */

	raw_spin_lock_irqsave(&rnp->lock, flags);
	delta = jiffies - rsp->jiffies_stall;
	if (delta < RCU_STALL_RAT_DELAY || !rcu_gp_in_progress(rsp)) {
		raw_spin_unlock_irqrestore(&rnp->lock, flags);
		return;
	}
	rsp->jiffies_stall = jiffies + RCU_SECONDS_TILL_STALL_RECHECK;

	/*
	 * Now rat on any tasks that got kicked up to the root rcu_node
	 * due to CPU offlining.
	 */
	rcu_print_task_stall(rnp);
	raw_spin_unlock_irqrestore(&rnp->lock, flags);

	/* OK, time to rat on our buddy... */

	printk(KERN_ERR "INFO: %s detected stalls on CPUs/tasks: {",
	       rsp->name);
	rcu_for_each_leaf_node(rsp, rnp) {
		raw_spin_lock_irqsave(&rnp->lock, flags);
		rcu_print_task_stall(rnp);
		raw_spin_unlock_irqrestore(&rnp->lock, flags);
		if (rnp->qsmask == 0)
			continue;
		for (cpu = 0; cpu <= rnp->grphi - rnp->grplo; cpu++)
			if (rnp->qsmask & (1UL << cpu))
				printk(" %d", rnp->grplo + cpu);
	}
	printk("} (detected by %d, t=%ld jiffies)\n",
	       smp_processor_id(), (long)(jiffies - rsp->gp_start));
	trigger_all_cpu_backtrace();

	/* If so configured, complain about tasks blocking the grace period. */

	rcu_print_detail_task_stall(rsp);

	force_quiescent_state(rsp, 0);  /* Kick them all. */
}

static void print_cpu_stall(struct rcu_state *rsp)
{
	unsigned long flags;
	struct rcu_node *rnp = rcu_get_root(rsp);

	printk(KERN_ERR "INFO: %s detected stall on CPU %d (t=%lu jiffies)\n",
	       rsp->name, smp_processor_id(), jiffies - rsp->gp_start);
	trigger_all_cpu_backtrace();

	raw_spin_lock_irqsave(&rnp->lock, flags);
	if (ULONG_CMP_GE(jiffies, rsp->jiffies_stall))
		rsp->jiffies_stall =
			jiffies + RCU_SECONDS_TILL_STALL_RECHECK;
	raw_spin_unlock_irqrestore(&rnp->lock, flags);

	set_need_resched();  /* kick ourselves to get things going. */
}

static void check_cpu_stall(struct rcu_state *rsp, struct rcu_data *rdp)
{
	long delta;
	struct rcu_node *rnp;

	if (rcu_cpu_stall_panicking)
		return;
	delta = jiffies - rsp->jiffies_stall;
	rnp = rdp->mynode;
	if ((rnp->qsmask & rdp->grpmask) && delta >= 0) {

		/* We haven't checked in, so go dump stack. */
		print_cpu_stall(rsp);

	} else if (rcu_gp_in_progress(rsp) && delta >= RCU_STALL_RAT_DELAY) {

		/* They had two time units to dump stack, so complain. */
		print_other_cpu_stall(rsp);
	}
}

static int rcu_panic(struct notifier_block *this, unsigned long ev, void *ptr)
{
	rcu_cpu_stall_panicking = 1;
	return NOTIFY_DONE;
}

static struct notifier_block rcu_panic_block = {
	.notifier_call = rcu_panic,
};

static void __init check_cpu_stall_init(void)
{
	atomic_notifier_chain_register(&panic_notifier_list, &rcu_panic_block);
}

#else /* #ifdef CONFIG_RCU_CPU_STALL_DETECTOR */

static void record_gp_stall_check_time(struct rcu_state *rsp)
{
}

static void check_cpu_stall(struct rcu_state *rsp, struct rcu_data *rdp)
{
}

static void __init check_cpu_stall_init(void)
{
}

#endif /* #else #ifdef CONFIG_RCU_CPU_STALL_DETECTOR */

static void __note_new_gpnum(struct rcu_state *rsp, struct rcu_node *rnp, struct rcu_data *rdp)
{
	if (rdp->gpnum != rnp->gpnum) {
		rdp->qs_pending = 1;
		rdp->passed_quiesc = 0;
		rdp->gpnum = rnp->gpnum;
	}
}

static void note_new_gpnum(struct rcu_state *rsp, struct rcu_data *rdp)
{
	unsigned long flags;
	struct rcu_node *rnp;

	local_irq_save(flags);
	rnp = rdp->mynode;
	if (rdp->gpnum == ACCESS_ONCE(rnp->gpnum) || /* outside lock. */
	    !raw_spin_trylock(&rnp->lock)) { /* irqs already off, so later. */
		local_irq_restore(flags);
		return;
	}
	__note_new_gpnum(rsp, rnp, rdp);
	raw_spin_unlock_irqrestore(&rnp->lock, flags);
}

static int
check_for_new_grace_period(struct rcu_state *rsp, struct rcu_data *rdp)
{
	unsigned long flags;
	int ret = 0;

	local_irq_save(flags);
	if (rdp->gpnum != rsp->gpnum) {
		note_new_gpnum(rsp, rdp);
		ret = 1;
	}
	local_irq_restore(flags);
	return ret;
}

static void
__rcu_process_gp_end(struct rcu_state *rsp, struct rcu_node *rnp, struct rcu_data *rdp)
{
	/* Did another grace period end? */
	if (rdp->completed != rnp->completed) {

		/* Advance callbacks.  No harm if list empty. */
		rdp->nxttail[RCU_DONE_TAIL] = rdp->nxttail[RCU_WAIT_TAIL];
		rdp->nxttail[RCU_WAIT_TAIL] = rdp->nxttail[RCU_NEXT_READY_TAIL];
		rdp->nxttail[RCU_NEXT_READY_TAIL] = rdp->nxttail[RCU_NEXT_TAIL];

		/* Remember that we saw this grace-period completion. */
		rdp->completed = rnp->completed;
	}
}

static void
rcu_process_gp_end(struct rcu_state *rsp, struct rcu_data *rdp)
{
	unsigned long flags;
	struct rcu_node *rnp;

	local_irq_save(flags);
	rnp = rdp->mynode;
	if (rdp->completed == ACCESS_ONCE(rnp->completed) || /* outside lock. */
	    !raw_spin_trylock(&rnp->lock)) { /* irqs already off, so later. */
		local_irq_restore(flags);
		return;
	}
	__rcu_process_gp_end(rsp, rnp, rdp);
	raw_spin_unlock_irqrestore(&rnp->lock, flags);
}

static void
rcu_start_gp_per_cpu(struct rcu_state *rsp, struct rcu_node *rnp, struct rcu_data *rdp)
{
	/* Prior grace period ended, so advance callbacks for current CPU. */
	__rcu_process_gp_end(rsp, rnp, rdp);

	/*
	 * Because this CPU just now started the new grace period, we know
	 * that all of its callbacks will be covered by this upcoming grace
	 * period, even the ones that were registered arbitrarily recently.
	 * Therefore, advance all outstanding callbacks to RCU_WAIT_TAIL.
	 *
	 * Other CPUs cannot be sure exactly when the grace period started.
	 * Therefore, their recently registered callbacks must pass through
	 * an additional RCU_NEXT_READY stage, so that they will be handled
	 * by the next RCU grace period.
	 */
	rdp->nxttail[RCU_NEXT_READY_TAIL] = rdp->nxttail[RCU_NEXT_TAIL];
	rdp->nxttail[RCU_WAIT_TAIL] = rdp->nxttail[RCU_NEXT_TAIL];

	/* Set state so that this CPU will detect the next quiescent state. */
	__note_new_gpnum(rsp, rnp, rdp);
}

static void
rcu_start_gp(struct rcu_state *rsp, unsigned long flags)
	__releases(rcu_get_root(rsp)->lock)
{
	struct rcu_data *rdp = rsp->rda[smp_processor_id()];
	struct rcu_node *rnp = rcu_get_root(rsp);

	if (!cpu_needs_another_gp(rsp, rdp) || rsp->fqs_active) {
		if (cpu_needs_another_gp(rsp, rdp))
			rsp->fqs_need_gp = 1;
		if (rnp->completed == rsp->completed) {
			raw_spin_unlock_irqrestore(&rnp->lock, flags);
			return;
		}
		raw_spin_unlock(&rnp->lock);	 /* irqs remain disabled. */

		/*
		 * Propagate new ->completed value to rcu_node structures
		 * so that other CPUs don't have to wait until the start
		 * of the next grace period to process their callbacks.
		 */
		rcu_for_each_node_breadth_first(rsp, rnp) {
			raw_spin_lock(&rnp->lock); /* irqs already disabled. */
			rnp->completed = rsp->completed;
			raw_spin_unlock(&rnp->lock); /* irqs remain disabled. */
		}
		local_irq_restore(flags);
		return;
	}

	/* Advance to a new grace period and initialize state. */
	rsp->gpnum++;
	WARN_ON_ONCE(rsp->signaled == RCU_GP_INIT);
	rsp->signaled = RCU_GP_INIT; /* Hold off force_quiescent_state. */
	rsp->jiffies_force_qs = jiffies + RCU_JIFFIES_TILL_FORCE_QS;
	record_gp_stall_check_time(rsp);

	/* Special-case the common single-level case. */
	if (NUM_RCU_NODES == 1) {
		rcu_preempt_check_blocked_tasks(rnp);
		rnp->qsmask = rnp->qsmaskinit;
		rnp->gpnum = rsp->gpnum;
		rnp->completed = rsp->completed;
		rsp->signaled = RCU_SIGNAL_INIT; /* force_quiescent_state OK. */
		rcu_start_gp_per_cpu(rsp, rnp, rdp);
		raw_spin_unlock_irqrestore(&rnp->lock, flags);
		return;
	}

	raw_spin_unlock(&rnp->lock);  /* leave irqs disabled. */


	/* Exclude any concurrent CPU-hotplug operations. */
	raw_spin_lock(&rsp->onofflock);  /* irqs already disabled. */

	/*
	 * Set the quiescent-state-needed bits in all the rcu_node
	 * structures for all currently online CPUs in breadth-first
	 * order, starting from the root rcu_node structure.  This
	 * operation relies on the layout of the hierarchy within the
	 * rsp->node[] array.  Note that other CPUs will access only
	 * the leaves of the hierarchy, which still indicate that no
	 * grace period is in progress, at least until the corresponding
	 * leaf node has been initialized.  In addition, we have excluded
	 * CPU-hotplug operations.
	 *
	 * Note that the grace period cannot complete until we finish
	 * the initialization process, as there will be at least one
	 * qsmask bit set in the root node until that time, namely the
	 * one corresponding to this CPU, due to the fact that we have
	 * irqs disabled.
	 */
	rcu_for_each_node_breadth_first(rsp, rnp) {
		raw_spin_lock(&rnp->lock);	/* irqs already disabled. */
		rcu_preempt_check_blocked_tasks(rnp);
		rnp->qsmask = rnp->qsmaskinit;
		rnp->gpnum = rsp->gpnum;
		rnp->completed = rsp->completed;
		if (rnp == rdp->mynode)
			rcu_start_gp_per_cpu(rsp, rnp, rdp);
		raw_spin_unlock(&rnp->lock);	/* irqs remain disabled. */
	}

	rnp = rcu_get_root(rsp);
	raw_spin_lock(&rnp->lock);		/* irqs already disabled. */
	rsp->signaled = RCU_SIGNAL_INIT; /* force_quiescent_state now OK. */
	raw_spin_unlock(&rnp->lock);		/* irqs remain disabled. */
	raw_spin_unlock_irqrestore(&rsp->onofflock, flags);
}

static void rcu_report_qs_rsp(struct rcu_state *rsp, unsigned long flags)
	__releases(rcu_get_root(rsp)->lock)
{
	WARN_ON_ONCE(!rcu_gp_in_progress(rsp));
	rsp->completed = rsp->gpnum;
	rsp->signaled = RCU_GP_IDLE;
	rcu_start_gp(rsp, flags);  /* releases root node's rnp->lock. */
}

static void
rcu_report_qs_rnp(unsigned long mask, struct rcu_state *rsp,
		  struct rcu_node *rnp, unsigned long flags)
	__releases(rnp->lock)
{
	struct rcu_node *rnp_c;

	/* Walk up the rcu_node hierarchy. */
	for (;;) {
		if (!(rnp->qsmask & mask)) {

			/* Our bit has already been cleared, so done. */
			raw_spin_unlock_irqrestore(&rnp->lock, flags);
			return;
		}
		rnp->qsmask &= ~mask;
		if (rnp->qsmask != 0 || rcu_preempted_readers(rnp)) {

			/* Other bits still set at this level, so done. */
			raw_spin_unlock_irqrestore(&rnp->lock, flags);
			return;
		}
		mask = rnp->grpmask;
		if (rnp->parent == NULL) {

			/* No more levels.  Exit loop holding root lock. */

			break;
		}
		raw_spin_unlock_irqrestore(&rnp->lock, flags);
		rnp_c = rnp;
		rnp = rnp->parent;
		raw_spin_lock_irqsave(&rnp->lock, flags);
		WARN_ON_ONCE(rnp_c->qsmask);
	}

	/*
	 * Get here if we are the last CPU to pass through a quiescent
	 * state for this grace period.  Invoke rcu_report_qs_rsp()
	 * to clean up and start the next grace period if one is needed.
	 */
	rcu_report_qs_rsp(rsp, flags); /* releases rnp->lock. */
}

static void
rcu_report_qs_rdp(int cpu, struct rcu_state *rsp, struct rcu_data *rdp, long lastcomp)
{
	unsigned long flags;
	unsigned long mask;
	struct rcu_node *rnp;

	rnp = rdp->mynode;
	raw_spin_lock_irqsave(&rnp->lock, flags);
	if (lastcomp != rnp->completed) {

		/*
		 * Someone beat us to it for this grace period, so leave.
		 * The race with GP start is resolved by the fact that we
		 * hold the leaf rcu_node lock, so that the per-CPU bits
		 * cannot yet be initialized -- so we would simply find our
		 * CPU's bit already cleared in rcu_report_qs_rnp() if this
		 * race occurred.
		 */
		rdp->passed_quiesc = 0;	/* try again later! */
		raw_spin_unlock_irqrestore(&rnp->lock, flags);
		return;
	}
	mask = rdp->grpmask;
	if ((rnp->qsmask & mask) == 0) {
		raw_spin_unlock_irqrestore(&rnp->lock, flags);
	} else {
		rdp->qs_pending = 0;

		/*
		 * This GP can't end until cpu checks in, so all of our
		 * callbacks can be processed during the next GP.
		 */
		rdp->nxttail[RCU_NEXT_READY_TAIL] = rdp->nxttail[RCU_NEXT_TAIL];

		rcu_report_qs_rnp(mask, rsp, rnp, flags); /* rlses rnp->lock */
	}
}

static void
rcu_check_quiescent_state(struct rcu_state *rsp, struct rcu_data *rdp)
{
	/* If there is now a new grace period, record and return. */
	if (check_for_new_grace_period(rsp, rdp))
		return;

	/*
	 * Does this CPU still need to do its part for current grace period?
	 * If no, return and let the other CPUs do their part as well.
	 */
	if (!rdp->qs_pending)
		return;

	/*
	 * Was there a quiescent state since the beginning of the grace
	 * period? If no, then exit and wait for the next call.
	 */
	if (!rdp->passed_quiesc)
		return;

	/*
	 * Tell RCU we are done (but rcu_report_qs_rdp() will be the
	 * judge of that).
	 */
	rcu_report_qs_rdp(rdp->cpu, rsp, rdp, rdp->passed_quiesc_completed);
}

#ifdef CONFIG_HOTPLUG_CPU

static void rcu_send_cbs_to_orphanage(struct rcu_state *rsp)
{
	int i;
	struct rcu_data *rdp = rsp->rda[smp_processor_id()];

	if (rdp->nxtlist == NULL)
		return;  /* irqs disabled, so comparison is stable. */
	raw_spin_lock(&rsp->onofflock);  /* irqs already disabled. */
	*rsp->orphan_cbs_tail = rdp->nxtlist;
	rsp->orphan_cbs_tail = rdp->nxttail[RCU_NEXT_TAIL];
	rdp->nxtlist = NULL;
	for (i = 0; i < RCU_NEXT_SIZE; i++)
		rdp->nxttail[i] = &rdp->nxtlist;
	rsp->orphan_qlen += rdp->qlen;
	rdp->qlen = 0;
	raw_spin_unlock(&rsp->onofflock);  /* irqs remain disabled. */
}

static void rcu_adopt_orphan_cbs(struct rcu_state *rsp)
{
	unsigned long flags;
	struct rcu_data *rdp;

	raw_spin_lock_irqsave(&rsp->onofflock, flags);
	rdp = rsp->rda[smp_processor_id()];
	if (rsp->orphan_cbs_list == NULL) {
		raw_spin_unlock_irqrestore(&rsp->onofflock, flags);
		return;
	}
	*rdp->nxttail[RCU_NEXT_TAIL] = rsp->orphan_cbs_list;
	rdp->nxttail[RCU_NEXT_TAIL] = rsp->orphan_cbs_tail;
	rdp->qlen += rsp->orphan_qlen;
	rsp->orphan_cbs_list = NULL;
	rsp->orphan_cbs_tail = &rsp->orphan_cbs_list;
	rsp->orphan_qlen = 0;
	raw_spin_unlock_irqrestore(&rsp->onofflock, flags);
}

static void __rcu_offline_cpu(int cpu, struct rcu_state *rsp)
{
	unsigned long flags;
	unsigned long mask;
	int need_report = 0;
	struct rcu_data *rdp = rsp->rda[cpu];
	struct rcu_node *rnp;

	/* Exclude any attempts to start a new grace period. */
	raw_spin_lock_irqsave(&rsp->onofflock, flags);

	/* Remove the outgoing CPU from the masks in the rcu_node hierarchy. */
	rnp = rdp->mynode;	/* this is the outgoing CPU's rnp. */
	mask = rdp->grpmask;	/* rnp->grplo is constant. */
	do {
		raw_spin_lock(&rnp->lock);	/* irqs already disabled. */
		rnp->qsmaskinit &= ~mask;
		if (rnp->qsmaskinit != 0) {
			if (rnp != rdp->mynode)
				raw_spin_unlock(&rnp->lock); /* irqs remain disabled. */
			break;
		}
		if (rnp == rdp->mynode)
			need_report = rcu_preempt_offline_tasks(rsp, rnp, rdp);
		else
			raw_spin_unlock(&rnp->lock); /* irqs remain disabled. */
		mask = rnp->grpmask;
		rnp = rnp->parent;
	} while (rnp != NULL);

	/*
	 * We still hold the leaf rcu_node structure lock here, and
	 * irqs are still disabled.  The reason for this subterfuge is
	 * because invoking rcu_report_unblock_qs_rnp() with ->onofflock
	 * held leads to deadlock.
	 */
	raw_spin_unlock(&rsp->onofflock); /* irqs remain disabled. */
	rnp = rdp->mynode;
	if (need_report & RCU_OFL_TASKS_NORM_GP)
		rcu_report_unblock_qs_rnp(rnp, flags);
	else
		raw_spin_unlock_irqrestore(&rnp->lock, flags);
	if (need_report & RCU_OFL_TASKS_EXP_GP)
		rcu_report_exp_rnp(rsp, rnp);

	rcu_adopt_orphan_cbs(rsp);
}

static void rcu_offline_cpu(int cpu)
{
	__rcu_offline_cpu(cpu, &rcu_sched_state);
	__rcu_offline_cpu(cpu, &rcu_bh_state);
	rcu_preempt_offline_cpu(cpu);
}

#else /* #ifdef CONFIG_HOTPLUG_CPU */

static void rcu_send_cbs_to_orphanage(struct rcu_state *rsp)
{
}

static void rcu_adopt_orphan_cbs(struct rcu_state *rsp)
{
}

static void rcu_offline_cpu(int cpu)
{
}

#endif /* #else #ifdef CONFIG_HOTPLUG_CPU */

static void rcu_do_batch(struct rcu_state *rsp, struct rcu_data *rdp)
{
	unsigned long flags;
	struct rcu_head *next, *list, **tail;
	int count;

	/* If no callbacks are ready, just return.*/
	if (!cpu_has_callbacks_ready_to_invoke(rdp))
		return;

	/*
	 * Extract the list of ready callbacks, disabling to prevent
	 * races with call_rcu() from interrupt handlers.
	 */
	local_irq_save(flags);
	list = rdp->nxtlist;
	rdp->nxtlist = *rdp->nxttail[RCU_DONE_TAIL];
	*rdp->nxttail[RCU_DONE_TAIL] = NULL;
	tail = rdp->nxttail[RCU_DONE_TAIL];
	for (count = RCU_NEXT_SIZE - 1; count >= 0; count--)
		if (rdp->nxttail[count] == rdp->nxttail[RCU_DONE_TAIL])
			rdp->nxttail[count] = &rdp->nxtlist;
	local_irq_restore(flags);

	/* Invoke callbacks. */
	count = 0;
	while (list) {
		next = list->next;
		prefetch(next);
		trace_rcu_tree_callback(list);
		list->func(list);
		list = next;
		if (++count >= rdp->blimit)
			break;
	}

	local_irq_save(flags);

	/* Update count, and requeue any remaining callbacks. */
	rdp->qlen -= count;
	if (list != NULL) {
		*tail = rdp->nxtlist;
		rdp->nxtlist = list;
		for (count = 0; count < RCU_NEXT_SIZE; count++)
			if (&rdp->nxtlist == rdp->nxttail[count])
				rdp->nxttail[count] = tail;
			else
				break;
	}

	/* Reinstate batch limit if we have worked down the excess. */
	if (rdp->blimit == LONG_MAX && rdp->qlen <= qlowmark)
		rdp->blimit = blimit;

	/* Reset ->qlen_last_fqs_check trigger if enough CBs have drained. */
	if (rdp->qlen == 0 && rdp->qlen_last_fqs_check != 0) {
		rdp->qlen_last_fqs_check = 0;
		rdp->n_force_qs_snap = rsp->n_force_qs;
	} else if (rdp->qlen < rdp->qlen_last_fqs_check - qhimark)
		rdp->qlen_last_fqs_check = rdp->qlen;

	local_irq_restore(flags);

	/* Re-raise the RCU softirq if there are callbacks remaining. */
	if (cpu_has_callbacks_ready_to_invoke(rdp))
		raise_softirq(RCU_SOFTIRQ);
}

void rcu_check_callbacks(int cpu, int user)
{
	if (user ||
	    (idle_cpu(cpu) && rcu_scheduler_active &&
	     !in_softirq() && hardirq_count() <= (1 << HARDIRQ_SHIFT))) {

		/*
		 * Get here if this CPU took its interrupt from user
		 * mode or from the idle loop, and if this is not a
		 * nested interrupt.  In this case, the CPU is in
		 * a quiescent state, so note it.
		 *
		 * No memory barrier is required here because both
		 * rcu_sched_qs() and rcu_bh_qs() reference only CPU-local
		 * variables that other CPUs neither access nor modify,
		 * at least not while the corresponding CPU is online.
		 */

		rcu_sched_qs(cpu);
		rcu_bh_qs(cpu);

	} else if (!in_softirq()) {

		/*
		 * Get here if this CPU did not take its interrupt from
		 * softirq, in other words, if it is not interrupting
		 * a rcu_bh read-side critical section.  This is an _bh
		 * critical section, so note it.
		 */

		rcu_bh_qs(cpu);
	}
	rcu_preempt_check_callbacks(cpu);
	if (rcu_pending(cpu))
		raise_softirq(RCU_SOFTIRQ);
}

#ifdef CONFIG_SMP

static void force_qs_rnp(struct rcu_state *rsp, int (*f)(struct rcu_data *))
{
	unsigned long bit;
	int cpu;
	unsigned long flags;
	unsigned long mask;
	struct rcu_node *rnp;

	rcu_for_each_leaf_node(rsp, rnp) {
		mask = 0;
		raw_spin_lock_irqsave(&rnp->lock, flags);
		if (!rcu_gp_in_progress(rsp)) {
			raw_spin_unlock_irqrestore(&rnp->lock, flags);
			return;
		}
		if (rnp->qsmask == 0) {
			raw_spin_unlock_irqrestore(&rnp->lock, flags);
			continue;
		}
		cpu = rnp->grplo;
		bit = 1;
		for (; cpu <= rnp->grphi; cpu++, bit <<= 1) {
			if ((rnp->qsmask & bit) != 0 && f(rsp->rda[cpu]))
				mask |= bit;
		}
		if (mask != 0) {

			/* rcu_report_qs_rnp() releases rnp->lock. */
			rcu_report_qs_rnp(mask, rsp, rnp, flags);
			continue;
		}
		raw_spin_unlock_irqrestore(&rnp->lock, flags);
	}
}

static void force_quiescent_state(struct rcu_state *rsp, int relaxed)
{
	unsigned long flags;
	struct rcu_node *rnp = rcu_get_root(rsp);

	if (!rcu_gp_in_progress(rsp))
		return;  /* No grace period in progress, nothing to force. */
	if (!raw_spin_trylock_irqsave(&rsp->fqslock, flags)) {
		rsp->n_force_qs_lh++; /* Inexact, can lose counts.  Tough! */
		return;	/* Someone else is already on the job. */
	}
	if (relaxed && ULONG_CMP_GE(rsp->jiffies_force_qs, jiffies))
		goto unlock_fqs_ret; /* no emergency and done recently. */
	rsp->n_force_qs++;
	raw_spin_lock(&rnp->lock);  /* irqs already disabled */
	rsp->jiffies_force_qs = jiffies + RCU_JIFFIES_TILL_FORCE_QS;
	if(!rcu_gp_in_progress(rsp)) {
		rsp->n_force_qs_ngp++;
		raw_spin_unlock(&rnp->lock);  /* irqs remain disabled */
		goto unlock_fqs_ret;  /* no GP in progress, time updated. */
	}
	rsp->fqs_active = 1;
	switch (rsp->signaled) {
	case RCU_GP_IDLE:
	case RCU_GP_INIT:

		break; /* grace period idle or initializing, ignore. */

	case RCU_SAVE_DYNTICK:
		if (RCU_SIGNAL_INIT != RCU_SAVE_DYNTICK)
			break; /* So gcc recognizes the dead code. */

		raw_spin_unlock(&rnp->lock);  /* irqs remain disabled */

		/* Record dyntick-idle state. */
		force_qs_rnp(rsp, dyntick_save_progress_counter);
		raw_spin_lock(&rnp->lock);  /* irqs already disabled */
		if (rcu_gp_in_progress(rsp))
			rsp->signaled = RCU_FORCE_QS;
		break;

	case RCU_FORCE_QS:

		/* Check dyntick-idle state, send IPI to laggarts. */
		raw_spin_unlock(&rnp->lock);  /* irqs remain disabled */
		force_qs_rnp(rsp, rcu_implicit_dynticks_qs);

		/* Leave state in case more forcing is required. */

		raw_spin_lock(&rnp->lock);  /* irqs already disabled */
		break;
	}
	rsp->fqs_active = 0;
	if (rsp->fqs_need_gp) {
		raw_spin_unlock(&rsp->fqslock); /* irqs remain disabled */
		rsp->fqs_need_gp = 0;
		rcu_start_gp(rsp, flags); /* releases rnp->lock */
		return;
	}
	raw_spin_unlock(&rnp->lock);  /* irqs remain disabled */
unlock_fqs_ret:
	raw_spin_unlock_irqrestore(&rsp->fqslock, flags);
}

#else /* #ifdef CONFIG_SMP */

static void force_quiescent_state(struct rcu_state *rsp, int relaxed)
{
	set_need_resched();
}

#endif /* #else #ifdef CONFIG_SMP */

static void
__rcu_process_callbacks(struct rcu_state *rsp, struct rcu_data *rdp)
{
	unsigned long flags;

	WARN_ON_ONCE(rdp->beenonline == 0);

	/*
	 * If an RCU GP has gone long enough, go check for dyntick
	 * idle CPUs and, if needed, send resched IPIs.
	 */
	if (ULONG_CMP_LT(ACCESS_ONCE(rsp->jiffies_force_qs), jiffies))
		force_quiescent_state(rsp, 1);

	/*
	 * Advance callbacks in response to end of earlier grace
	 * period that some other CPU ended.
	 */
	rcu_process_gp_end(rsp, rdp);

	/* Update RCU state based on any recent quiescent states. */
	rcu_check_quiescent_state(rsp, rdp);

	/* Does this CPU require a not-yet-started grace period? */
	if (cpu_needs_another_gp(rsp, rdp)) {
		raw_spin_lock_irqsave(&rcu_get_root(rsp)->lock, flags);
		rcu_start_gp(rsp, flags);  /* releases above lock */
	}

	/* If there are callbacks ready, invoke them. */
	rcu_do_batch(rsp, rdp);
}

static void rcu_process_callbacks(struct softirq_action *unused)
{
	/*
	 * Memory references from any prior RCU read-side critical sections
	 * executed by the interrupted code must be seen before any RCU
	 * grace-period manipulations below.
	 */
	smp_mb(); /* See above block comment. */

	__rcu_process_callbacks(&rcu_sched_state,
				&__get_cpu_var(rcu_sched_data));
	__rcu_process_callbacks(&rcu_bh_state, &__get_cpu_var(rcu_bh_data));
	rcu_preempt_process_callbacks();

	/*
	 * Memory references from any later RCU read-side critical sections
	 * executed by the interrupted code must be seen after any RCU
	 * grace-period manipulations above.
	 */
	smp_mb(); /* See above block comment. */

	/* If we are last CPU on way to dyntick-idle mode, accelerate it. */
	rcu_needs_cpu_flush();
}

static void
__call_rcu(struct rcu_head *head, void (*func)(struct rcu_head *rcu),
	   struct rcu_state *rsp)
{
	unsigned long flags;
	struct rcu_data *rdp;

	head->func = func;
	head->next = NULL;

	smp_mb(); /* Ensure RCU update seen before callback registry. */

	/*
	 * Opportunistically note grace-period endings and beginnings.
	 * Note that we might see a beginning right after we see an
	 * end, but never vice versa, since this CPU has to pass through
	 * a quiescent state betweentimes.
	 */
	local_irq_save(flags);
	rdp = rsp->rda[smp_processor_id()];
	rcu_process_gp_end(rsp, rdp);
	check_for_new_grace_period(rsp, rdp);

	/* Add the callback to our list. */
	*rdp->nxttail[RCU_NEXT_TAIL] = head;
	rdp->nxttail[RCU_NEXT_TAIL] = &head->next;

	/* Start a new grace period if one not already started. */
	if (!rcu_gp_in_progress(rsp)) {
		unsigned long nestflag;
		struct rcu_node *rnp_root = rcu_get_root(rsp);

		raw_spin_lock_irqsave(&rnp_root->lock, nestflag);
		rcu_start_gp(rsp, nestflag);  /* releases rnp_root->lock. */
	}

	/*
	 * Force the grace period if too many callbacks or too long waiting.
	 * Enforce hysteresis, and don't invoke force_quiescent_state()
	 * if some other CPU has recently done so.  Also, don't bother
	 * invoking force_quiescent_state() if the newly enqueued callback
	 * is the only one waiting for a grace period to complete.
	 */
	if (unlikely(++rdp->qlen > rdp->qlen_last_fqs_check + qhimark)) {
		rdp->blimit = LONG_MAX;
		if (rsp->n_force_qs == rdp->n_force_qs_snap &&
		    *rdp->nxttail[RCU_DONE_TAIL] != head)
			force_quiescent_state(rsp, 0);
		rdp->n_force_qs_snap = rsp->n_force_qs;
		rdp->qlen_last_fqs_check = rdp->qlen;
	} else if (ULONG_CMP_LT(ACCESS_ONCE(rsp->jiffies_force_qs), jiffies))
		force_quiescent_state(rsp, 1);
	local_irq_restore(flags);
}

void call_rcu_sched(struct rcu_head *head, void (*func)(struct rcu_head *rcu))
{
	__call_rcu(head, func, &rcu_sched_state);
}
EXPORT_SYMBOL_GPL(call_rcu_sched);

void call_rcu_bh(struct rcu_head *head, void (*func)(struct rcu_head *rcu))
{
	trace_rcu_tree_call_rcu_bh(head, _RET_IP_);
	__call_rcu(head, func, &rcu_bh_state);
}
EXPORT_SYMBOL_GPL(call_rcu_bh);

void synchronize_sched(void)
{
	struct rcu_synchronize rcu;

	if (rcu_blocking_is_gp())
		return;

	init_rcu_head_on_stack(&rcu.head);
	init_completion(&rcu.completion);
	/* Will wake me after RCU finished. */
	call_rcu_sched(&rcu.head, wakeme_after_rcu);
	/* Wait for it. */
	wait_for_completion(&rcu.completion);
	destroy_rcu_head_on_stack(&rcu.head);
}
EXPORT_SYMBOL_GPL(synchronize_sched);

void synchronize_rcu_bh(void)
{
	struct rcu_synchronize rcu;

	if (rcu_blocking_is_gp())
		return;

	init_rcu_head_on_stack(&rcu.head);
	init_completion(&rcu.completion);
	/* Will wake me after RCU finished. */
	call_rcu_bh(&rcu.head, wakeme_after_rcu);
	/* Wait for it. */
	wait_for_completion(&rcu.completion);
	destroy_rcu_head_on_stack(&rcu.head);
}
EXPORT_SYMBOL_GPL(synchronize_rcu_bh);

static int __rcu_pending(struct rcu_state *rsp, struct rcu_data *rdp)
{
	struct rcu_node *rnp = rdp->mynode;

	rdp->n_rcu_pending++;

	/* Check for CPU stalls, if enabled. */
	check_cpu_stall(rsp, rdp);

	/* Is the RCU core waiting for a quiescent state from this CPU? */
	if (rdp->qs_pending && !rdp->passed_quiesc) {

		/*
		 * If force_quiescent_state() coming soon and this CPU
		 * needs a quiescent state, and this is either RCU-sched
		 * or RCU-bh, force a local reschedule.
		 */
		rdp->n_rp_qs_pending++;
		if (!rdp->preemptable &&
		    ULONG_CMP_LT(ACCESS_ONCE(rsp->jiffies_force_qs) - 1,
				 jiffies))
			set_need_resched();
	} else if (rdp->qs_pending && rdp->passed_quiesc) {
		rdp->n_rp_report_qs++;
		return 1;
	}

	/* Does this CPU have callbacks ready to invoke? */
	if (cpu_has_callbacks_ready_to_invoke(rdp)) {
		rdp->n_rp_cb_ready++;
		return 1;
	}

	/* Has RCU gone idle with this CPU needing another grace period? */
	if (cpu_needs_another_gp(rsp, rdp)) {
		rdp->n_rp_cpu_needs_gp++;
		return 1;
	}

	/* Has another RCU grace period completed?  */
	if (ACCESS_ONCE(rnp->completed) != rdp->completed) { /* outside lock */
		rdp->n_rp_gp_completed++;
		return 1;
	}

	/* Has a new RCU grace period started? */
	if (ACCESS_ONCE(rnp->gpnum) != rdp->gpnum) { /* outside lock */
		rdp->n_rp_gp_started++;
		return 1;
	}

	/* Has an RCU GP gone long enough to send resched IPIs &c? */
	if (rcu_gp_in_progress(rsp) &&
	    ULONG_CMP_LT(ACCESS_ONCE(rsp->jiffies_force_qs), jiffies)) {
		rdp->n_rp_need_fqs++;
		return 1;
	}

	/* nothing to do */
	rdp->n_rp_need_nothing++;
	return 0;
}

static int rcu_pending(int cpu)
{
	return __rcu_pending(&rcu_sched_state, &per_cpu(rcu_sched_data, cpu)) ||
	       __rcu_pending(&rcu_bh_state, &per_cpu(rcu_bh_data, cpu)) ||
	       rcu_preempt_pending(cpu);
}

static int rcu_needs_cpu_quick_check(int cpu)
{
	/* RCU callbacks either ready or pending? */
	return per_cpu(rcu_sched_data, cpu).nxtlist ||
	       per_cpu(rcu_bh_data, cpu).nxtlist ||
	       rcu_preempt_needs_cpu(cpu);
}

static DEFINE_PER_CPU(struct rcu_head, rcu_barrier_head) = {NULL};
static atomic_t rcu_barrier_cpu_count;
static DEFINE_MUTEX(rcu_barrier_mutex);
static struct completion rcu_barrier_completion;

static void rcu_barrier_callback(struct rcu_head *notused)
{
	if (atomic_dec_and_test(&rcu_barrier_cpu_count))
		complete(&rcu_barrier_completion);
}

static void rcu_barrier_func(void *type)
{
	int cpu = smp_processor_id();
	struct rcu_head *head = &per_cpu(rcu_barrier_head, cpu);
	void (*call_rcu_func)(struct rcu_head *head,
			      void (*func)(struct rcu_head *head));

	atomic_inc(&rcu_barrier_cpu_count);
	call_rcu_func = type;
	call_rcu_func(head, rcu_barrier_callback);
}

static void _rcu_barrier(struct rcu_state *rsp,
			 void (*call_rcu_func)(struct rcu_head *head,
					       void (*func)(struct rcu_head *head)))
{
	BUG_ON(in_interrupt());
	/* Take mutex to serialize concurrent rcu_barrier() requests. */
	mutex_lock(&rcu_barrier_mutex);
	init_completion(&rcu_barrier_completion);
	/*
	 * Initialize rcu_barrier_cpu_count to 1, then invoke
	 * rcu_barrier_func() on each CPU, so that each CPU also has
	 * incremented rcu_barrier_cpu_count.  Only then is it safe to
	 * decrement rcu_barrier_cpu_count -- otherwise the first CPU
	 * might complete its grace period before all of the other CPUs
	 * did their increment, causing this function to return too
	 * early.
	 */
	atomic_set(&rcu_barrier_cpu_count, 1);
	preempt_disable(); /* stop CPU_DYING from filling orphan_cbs_list */
	rcu_adopt_orphan_cbs(rsp);
	on_each_cpu(rcu_barrier_func, (void *)call_rcu_func, 1);
	preempt_enable(); /* CPU_DYING can again fill orphan_cbs_list */
	if (atomic_dec_and_test(&rcu_barrier_cpu_count))
		complete(&rcu_barrier_completion);
	wait_for_completion(&rcu_barrier_completion);
	mutex_unlock(&rcu_barrier_mutex);
}

void rcu_barrier_bh(void)
{
	_rcu_barrier(&rcu_bh_state, call_rcu_bh);
}
EXPORT_SYMBOL_GPL(rcu_barrier_bh);

void rcu_barrier_sched(void)
{
	_rcu_barrier(&rcu_sched_state, call_rcu_sched);
}
EXPORT_SYMBOL_GPL(rcu_barrier_sched);

static void __init
rcu_boot_init_percpu_data(int cpu, struct rcu_state *rsp)
{
	unsigned long flags;
	int i;
	struct rcu_data *rdp = rsp->rda[cpu];
	struct rcu_node *rnp = rcu_get_root(rsp);

	/* Set up local state, ensuring consistent view of global state. */
	raw_spin_lock_irqsave(&rnp->lock, flags);
	rdp->grpmask = 1UL << (cpu - rdp->mynode->grplo);
	rdp->nxtlist = NULL;
	for (i = 0; i < RCU_NEXT_SIZE; i++)
		rdp->nxttail[i] = &rdp->nxtlist;
	rdp->qlen = 0;
#ifdef CONFIG_NO_HZ
	rdp->dynticks = &per_cpu(rcu_dynticks, cpu);
#endif /* #ifdef CONFIG_NO_HZ */
	rdp->cpu = cpu;
	raw_spin_unlock_irqrestore(&rnp->lock, flags);
}

static void __cpuinit
rcu_init_percpu_data(int cpu, struct rcu_state *rsp, int preemptable)
{
	unsigned long flags;
	unsigned long mask;
	struct rcu_data *rdp = rsp->rda[cpu];
	struct rcu_node *rnp = rcu_get_root(rsp);

	/* Set up local state, ensuring consistent view of global state. */
	raw_spin_lock_irqsave(&rnp->lock, flags);
	rdp->passed_quiesc = 0;  /* We could be racing with new GP, */
	rdp->qs_pending = 1;	 /*  so set up to respond to current GP. */
	rdp->beenonline = 1;	 /* We have now been online. */
	rdp->preemptable = preemptable;
	rdp->qlen_last_fqs_check = 0;
	rdp->n_force_qs_snap = rsp->n_force_qs;
	rdp->blimit = blimit;
	raw_spin_unlock(&rnp->lock);		/* irqs remain disabled. */

	/*
	 * A new grace period might start here.  If so, we won't be part
	 * of it, but that is OK, as we are currently in a quiescent state.
	 */

	/* Exclude any attempts to start a new GP on large systems. */
	raw_spin_lock(&rsp->onofflock);		/* irqs already disabled. */

	/* Add CPU to rcu_node bitmasks. */
	rnp = rdp->mynode;
	mask = rdp->grpmask;
	do {
		/* Exclude any attempts to start a new GP on small systems. */
		raw_spin_lock(&rnp->lock);	/* irqs already disabled. */
		rnp->qsmaskinit |= mask;
		mask = rnp->grpmask;
		if (rnp == rdp->mynode) {
			rdp->gpnum = rnp->completed; /* if GP in progress... */
			rdp->completed = rnp->completed;
			rdp->passed_quiesc_completed = rnp->completed - 1;
		}
		raw_spin_unlock(&rnp->lock); /* irqs already disabled. */
		rnp = rnp->parent;
	} while (rnp != NULL && !(rnp->qsmaskinit & mask));

	raw_spin_unlock_irqrestore(&rsp->onofflock, flags);
}

static void __cpuinit rcu_online_cpu(int cpu)
{
	rcu_init_percpu_data(cpu, &rcu_sched_state, 0);
	rcu_init_percpu_data(cpu, &rcu_bh_state, 0);
	rcu_preempt_init_percpu_data(cpu);
}

static int __cpuinit rcu_cpu_notify(struct notifier_block *self,
				    unsigned long action, void *hcpu)
{
	long cpu = (long)hcpu;

	switch (action) {
	case CPU_UP_PREPARE:
	case CPU_UP_PREPARE_FROZEN:
		rcu_online_cpu(cpu);
		break;
	case CPU_DYING:
	case CPU_DYING_FROZEN:
		/*
		 * preempt_disable() in _rcu_barrier() prevents stop_machine(),
		 * so when "on_each_cpu(rcu_barrier_func, (void *)type, 1);"
		 * returns, all online cpus have queued rcu_barrier_func().
		 * The dying CPU clears its cpu_online_mask bit and
		 * moves all of its RCU callbacks to ->orphan_cbs_list
		 * in the context of stop_machine(), so subsequent calls
		 * to _rcu_barrier() will adopt these callbacks and only
		 * then queue rcu_barrier_func() on all remaining CPUs.
		 */
		rcu_send_cbs_to_orphanage(&rcu_bh_state);
		rcu_send_cbs_to_orphanage(&rcu_sched_state);
		rcu_preempt_send_cbs_to_orphanage();
		break;
	case CPU_DEAD:
	case CPU_DEAD_FROZEN:
	case CPU_UP_CANCELED:
	case CPU_UP_CANCELED_FROZEN:
		rcu_offline_cpu(cpu);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

void rcu_scheduler_starting(void)
{
	WARN_ON(num_online_cpus() != 1);
	WARN_ON(nr_context_switches() > 0);
	rcu_scheduler_active = 1;
}

#ifdef CONFIG_RCU_FANOUT_EXACT
static void __init rcu_init_levelspread(struct rcu_state *rsp)
{
	int i;

	for (i = NUM_RCU_LVLS - 1; i >= 0; i--)
		rsp->levelspread[i] = CONFIG_RCU_FANOUT;
}
#else /* #ifdef CONFIG_RCU_FANOUT_EXACT */
static void __init rcu_init_levelspread(struct rcu_state *rsp)
{
	int ccur;
	int cprv;
	int i;

	cprv = NR_CPUS;
	for (i = NUM_RCU_LVLS - 1; i >= 0; i--) {
		ccur = rsp->levelcnt[i];
		rsp->levelspread[i] = (cprv + ccur - 1) / ccur;
		cprv = ccur;
	}
}
#endif /* #else #ifdef CONFIG_RCU_FANOUT_EXACT */

static void __init rcu_init_one(struct rcu_state *rsp)
{
	static char *buf[] = { "rcu_node_level_0",
			       "rcu_node_level_1",
			       "rcu_node_level_2",
			       "rcu_node_level_3" };  /* Match MAX_RCU_LVLS */
	int cpustride = 1;
	int i;
	int j;
	struct rcu_node *rnp;

	BUILD_BUG_ON(MAX_RCU_LVLS > ARRAY_SIZE(buf));  /* Fix buf[] init! */

	/* Initialize the level-tracking arrays. */

	for (i = 1; i < NUM_RCU_LVLS; i++)
		rsp->level[i] = rsp->level[i - 1] + rsp->levelcnt[i - 1];
	rcu_init_levelspread(rsp);

	/* Initialize the elements themselves, starting from the leaves. */

	for (i = NUM_RCU_LVLS - 1; i >= 0; i--) {
		cpustride *= rsp->levelspread[i];
		rnp = rsp->level[i];
		for (j = 0; j < rsp->levelcnt[i]; j++, rnp++) {
			raw_spin_lock_init(&rnp->lock);
			lockdep_set_class_and_name(&rnp->lock,
						   &rcu_node_class[i], buf[i]);
			rnp->gpnum = 0;
			rnp->qsmask = 0;
			rnp->qsmaskinit = 0;
			rnp->grplo = j * cpustride;
			rnp->grphi = (j + 1) * cpustride - 1;
			if (rnp->grphi >= NR_CPUS)
				rnp->grphi = NR_CPUS - 1;
			if (i == 0) {
				rnp->grpnum = 0;
				rnp->grpmask = 0;
				rnp->parent = NULL;
			} else {
				rnp->grpnum = j % rsp->levelspread[i - 1];
				rnp->grpmask = 1UL << rnp->grpnum;
				rnp->parent = rsp->level[i - 1] +
					      j / rsp->levelspread[i - 1];
			}
			rnp->level = i;
			INIT_LIST_HEAD(&rnp->blocked_tasks[0]);
			INIT_LIST_HEAD(&rnp->blocked_tasks[1]);
			INIT_LIST_HEAD(&rnp->blocked_tasks[2]);
			INIT_LIST_HEAD(&rnp->blocked_tasks[3]);
		}
	}

	rnp = rsp->level[NUM_RCU_LVLS - 1];
	for_each_possible_cpu(i) {
		while (i > rnp->grphi)
			rnp++;
		rsp->rda[i]->mynode = rnp;
		rcu_boot_init_percpu_data(i, rsp);
	}
}

#define RCU_INIT_FLAVOR(rsp, rcu_data) \
do { \
	int i; \
	\
	for_each_possible_cpu(i) { \
		(rsp)->rda[i] = &per_cpu(rcu_data, i); \
	} \
	rcu_init_one(rsp); \
} while (0)

void __init rcu_init(void)
{
	int cpu;

	rcu_bootup_announce();
	RCU_INIT_FLAVOR(&rcu_sched_state, rcu_sched_data);
	RCU_INIT_FLAVOR(&rcu_bh_state, rcu_bh_data);
	__rcu_init_preempt();
	open_softirq(RCU_SOFTIRQ, rcu_process_callbacks);

	/*
	 * We don't need protection against CPU-hotplug here because
	 * this is called early in boot, before either interrupts
	 * or the scheduler are operational.
	 */
	cpu_notifier(rcu_cpu_notify, 0);
	for_each_online_cpu(cpu)
		rcu_cpu_notify(NULL, CPU_UP_PREPARE, (void *)(long)cpu);
	check_cpu_stall_init();
}

#include "rcutree_plugin.h"
