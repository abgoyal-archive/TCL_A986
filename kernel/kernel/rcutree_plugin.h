

#include <linux/delay.h>

static void __init rcu_bootup_announce_oddness(void)
{
#ifdef CONFIG_RCU_TRACE
	printk(KERN_INFO "\tRCU debugfs-based tracing is enabled.\n");
#endif
#if (defined(CONFIG_64BIT) && CONFIG_RCU_FANOUT != 64) || (!defined(CONFIG_64BIT) && CONFIG_RCU_FANOUT != 32)
	printk(KERN_INFO "\tCONFIG_RCU_FANOUT set to non-default value of %d\n",
	       CONFIG_RCU_FANOUT);
#endif
#ifdef CONFIG_RCU_FANOUT_EXACT
	printk(KERN_INFO "\tHierarchical RCU autobalancing is disabled.\n");
#endif
#ifdef CONFIG_RCU_FAST_NO_HZ
	printk(KERN_INFO
	       "\tRCU dyntick-idle grace-period acceleration is enabled.\n");
#endif
#ifdef CONFIG_PROVE_RCU
	printk(KERN_INFO "\tRCU lockdep checking is enabled.\n");
#endif
#ifdef CONFIG_RCU_TORTURE_TEST_RUNNABLE
	printk(KERN_INFO "\tRCU torture testing starts during boot.\n");
#endif
#ifndef CONFIG_RCU_CPU_STALL_DETECTOR
	printk(KERN_INFO
	       "\tRCU-based detection of stalled CPUs is disabled.\n");
#endif
#ifndef CONFIG_RCU_CPU_STALL_VERBOSE
	printk(KERN_INFO "\tVerbose stalled-CPUs detection is disabled.\n");
#endif
#if NUM_RCU_LVL_4 != 0
	printk(KERN_INFO "\tExperimental four-level hierarchy is enabled.\n");
#endif
}

#ifdef CONFIG_TREE_PREEMPT_RCU

struct rcu_state rcu_preempt_state = RCU_STATE_INITIALIZER(rcu_preempt_state);
DEFINE_PER_CPU(struct rcu_data, rcu_preempt_data);

static int rcu_preempted_readers_exp(struct rcu_node *rnp);

static void __init rcu_bootup_announce(void)
{
	printk(KERN_INFO "Preemptable hierarchical RCU implementation.\n");
	rcu_bootup_announce_oddness();
}

long rcu_batches_completed_preempt(void)
{
	return rcu_preempt_state.completed;
}
EXPORT_SYMBOL_GPL(rcu_batches_completed_preempt);

long rcu_batches_completed(void)
{
	return rcu_batches_completed_preempt();
}
EXPORT_SYMBOL_GPL(rcu_batches_completed);

void rcu_force_quiescent_state(void)
{
	force_quiescent_state(&rcu_preempt_state, 0);
}
EXPORT_SYMBOL_GPL(rcu_force_quiescent_state);

static void rcu_preempt_qs(int cpu)
{
	struct rcu_data *rdp = &per_cpu(rcu_preempt_data, cpu);

	rdp->passed_quiesc_completed = rdp->gpnum - 1;
	barrier();
	rdp->passed_quiesc = 1;
	current->rcu_read_unlock_special &= ~RCU_READ_UNLOCK_NEED_QS;
}

static void rcu_preempt_note_context_switch(int cpu)
{
	struct task_struct *t = current;
	unsigned long flags;
	int phase;
	struct rcu_data *rdp;
	struct rcu_node *rnp;

	if (t->rcu_read_lock_nesting &&
	    (t->rcu_read_unlock_special & RCU_READ_UNLOCK_BLOCKED) == 0) {

		/* Possibly blocking in an RCU read-side critical section. */
		rdp = rcu_preempt_state.rda[cpu];
		rnp = rdp->mynode;
		raw_spin_lock_irqsave(&rnp->lock, flags);
		t->rcu_read_unlock_special |= RCU_READ_UNLOCK_BLOCKED;
		t->rcu_blocked_node = rnp;

		/*
		 * If this CPU has already checked in, then this task
		 * will hold up the next grace period rather than the
		 * current grace period.  Queue the task accordingly.
		 * If the task is queued for the current grace period
		 * (i.e., this CPU has not yet passed through a quiescent
		 * state for the current grace period), then as long
		 * as that task remains queued, the current grace period
		 * cannot end.
		 *
		 * But first, note that the current CPU must still be
		 * on line!
		 */
		WARN_ON_ONCE((rdp->grpmask & rnp->qsmaskinit) == 0);
		WARN_ON_ONCE(!list_empty(&t->rcu_node_entry));
		phase = (rnp->gpnum + !(rnp->qsmask & rdp->grpmask)) & 0x1;
		list_add(&t->rcu_node_entry, &rnp->blocked_tasks[phase]);
		raw_spin_unlock_irqrestore(&rnp->lock, flags);
	}

	/*
	 * Either we were not in an RCU read-side critical section to
	 * begin with, or we have now recorded that critical section
	 * globally.  Either way, we can now note a quiescent state
	 * for this CPU.  Again, if we were in an RCU read-side critical
	 * section, and if that critical section was blocking the current
	 * grace period, then the fact that the task has been enqueued
	 * means that we continue to block the current grace period.
	 */
	local_irq_save(flags);
	rcu_preempt_qs(cpu);
	local_irq_restore(flags);
}

void __rcu_read_lock(void)
{
	ACCESS_ONCE(current->rcu_read_lock_nesting)++;
	barrier();  /* needed if we ever invoke rcu_read_lock in rcutree.c */
}
EXPORT_SYMBOL_GPL(__rcu_read_lock);

static int rcu_preempted_readers(struct rcu_node *rnp)
{
	int phase = rnp->gpnum & 0x1;

	return !list_empty(&rnp->blocked_tasks[phase]) ||
	       !list_empty(&rnp->blocked_tasks[phase + 2]);
}

static void rcu_report_unblock_qs_rnp(struct rcu_node *rnp, unsigned long flags)
	__releases(rnp->lock)
{
	unsigned long mask;
	struct rcu_node *rnp_p;

	if (rnp->qsmask != 0 || rcu_preempted_readers(rnp)) {
		raw_spin_unlock_irqrestore(&rnp->lock, flags);
		return;  /* Still need more quiescent states! */
	}

	rnp_p = rnp->parent;
	if (rnp_p == NULL) {
		/*
		 * Either there is only one rcu_node in the tree,
		 * or tasks were kicked up to root rcu_node due to
		 * CPUs going offline.
		 */
		rcu_report_qs_rsp(&rcu_preempt_state, flags);
		return;
	}

	/* Report up the rest of the hierarchy. */
	mask = rnp->grpmask;
	raw_spin_unlock(&rnp->lock);	/* irqs remain disabled. */
	raw_spin_lock(&rnp_p->lock);	/* irqs already disabled. */
	rcu_report_qs_rnp(mask, &rcu_preempt_state, rnp_p, flags);
}

static void rcu_read_unlock_special(struct task_struct *t)
{
	int empty;
	int empty_exp;
	unsigned long flags;
	struct rcu_node *rnp;
	int special;

	/* NMI handlers cannot block and cannot safely manipulate state. */
	if (in_nmi())
		return;

	local_irq_save(flags);

	/*
	 * If RCU core is waiting for this CPU to exit critical section,
	 * let it know that we have done so.
	 */
	special = t->rcu_read_unlock_special;
	if (special & RCU_READ_UNLOCK_NEED_QS) {
		rcu_preempt_qs(smp_processor_id());
	}

	/* Hardware IRQ handlers cannot block. */
	if (in_irq()) {
		local_irq_restore(flags);
		return;
	}

	/* Clean up if blocked during RCU read-side critical section. */
	if (special & RCU_READ_UNLOCK_BLOCKED) {
		t->rcu_read_unlock_special &= ~RCU_READ_UNLOCK_BLOCKED;

		/*
		 * Remove this task from the list it blocked on.  The
		 * task can migrate while we acquire the lock, but at
		 * most one time.  So at most two passes through loop.
		 */
		for (;;) {
			rnp = t->rcu_blocked_node;
			raw_spin_lock(&rnp->lock);  /* irqs already disabled. */
			if (rnp == t->rcu_blocked_node)
				break;
			raw_spin_unlock(&rnp->lock); /* irqs remain disabled. */
		}
		empty = !rcu_preempted_readers(rnp);
		empty_exp = !rcu_preempted_readers_exp(rnp);
		smp_mb(); /* ensure expedited fastpath sees end of RCU c-s. */
		list_del_init(&t->rcu_node_entry);
		t->rcu_blocked_node = NULL;

		/*
		 * If this was the last task on the current list, and if
		 * we aren't waiting on any CPUs, report the quiescent state.
		 * Note that rcu_report_unblock_qs_rnp() releases rnp->lock.
		 */
		if (empty)
			raw_spin_unlock_irqrestore(&rnp->lock, flags);
		else
			rcu_report_unblock_qs_rnp(rnp, flags);

		/*
		 * If this was the last task on the expedited lists,
		 * then we need to report up the rcu_node hierarchy.
		 */
		if (!empty_exp && !rcu_preempted_readers_exp(rnp))
			rcu_report_exp_rnp(&rcu_preempt_state, rnp);
	} else {
		local_irq_restore(flags);
	}
}

void __rcu_read_unlock(void)
{
	struct task_struct *t = current;

	barrier();  /* needed if we ever invoke rcu_read_unlock in rcutree.c */
	if (--ACCESS_ONCE(t->rcu_read_lock_nesting) == 0 &&
	    unlikely(ACCESS_ONCE(t->rcu_read_unlock_special)))
		rcu_read_unlock_special(t);
#ifdef CONFIG_PROVE_LOCKING
	WARN_ON_ONCE(ACCESS_ONCE(t->rcu_read_lock_nesting) < 0);
#endif /* #ifdef CONFIG_PROVE_LOCKING */
}
EXPORT_SYMBOL_GPL(__rcu_read_unlock);

#ifdef CONFIG_RCU_CPU_STALL_DETECTOR

#ifdef CONFIG_RCU_CPU_STALL_VERBOSE

static void rcu_print_detail_task_stall_rnp(struct rcu_node *rnp)
{
	unsigned long flags;
	struct list_head *lp;
	int phase;
	struct task_struct *t;

	if (rcu_preempted_readers(rnp)) {
		raw_spin_lock_irqsave(&rnp->lock, flags);
		phase = rnp->gpnum & 0x1;
		lp = &rnp->blocked_tasks[phase];
		list_for_each_entry(t, lp, rcu_node_entry)
			sched_show_task(t);
		raw_spin_unlock_irqrestore(&rnp->lock, flags);
	}
}

static void rcu_print_detail_task_stall(struct rcu_state *rsp)
{
	struct rcu_node *rnp = rcu_get_root(rsp);

	rcu_print_detail_task_stall_rnp(rnp);
	rcu_for_each_leaf_node(rsp, rnp)
		rcu_print_detail_task_stall_rnp(rnp);
}

#else /* #ifdef CONFIG_RCU_CPU_STALL_VERBOSE */

static void rcu_print_detail_task_stall(struct rcu_state *rsp)
{
}

#endif /* #else #ifdef CONFIG_RCU_CPU_STALL_VERBOSE */

static void rcu_print_task_stall(struct rcu_node *rnp)
{
	struct list_head *lp;
	int phase;
	struct task_struct *t;

	if (rcu_preempted_readers(rnp)) {
		phase = rnp->gpnum & 0x1;
		lp = &rnp->blocked_tasks[phase];
		list_for_each_entry(t, lp, rcu_node_entry)
			printk(" P%d", t->pid);
	}
}

#endif /* #ifdef CONFIG_RCU_CPU_STALL_DETECTOR */

static void rcu_preempt_check_blocked_tasks(struct rcu_node *rnp)
{
	WARN_ON_ONCE(rcu_preempted_readers(rnp));
	WARN_ON_ONCE(rnp->qsmask);
}

#ifdef CONFIG_HOTPLUG_CPU

static int rcu_preempt_offline_tasks(struct rcu_state *rsp,
				     struct rcu_node *rnp,
				     struct rcu_data *rdp)
{
	int i;
	struct list_head *lp;
	struct list_head *lp_root;
	int retval = 0;
	struct rcu_node *rnp_root = rcu_get_root(rsp);
	struct task_struct *tp;

	if (rnp == rnp_root) {
		WARN_ONCE(1, "Last CPU thought to be offlined?");
		return 0;  /* Shouldn't happen: at least one CPU online. */
	}
	WARN_ON_ONCE(rnp != rdp->mynode &&
		     (!list_empty(&rnp->blocked_tasks[0]) ||
		      !list_empty(&rnp->blocked_tasks[1]) ||
		      !list_empty(&rnp->blocked_tasks[2]) ||
		      !list_empty(&rnp->blocked_tasks[3])));

	/*
	 * Move tasks up to root rcu_node.  Rely on the fact that the
	 * root rcu_node can be at most one ahead of the rest of the
	 * rcu_nodes in terms of gp_num value.  This fact allows us to
	 * move the blocked_tasks[] array directly, element by element.
	 */
	if (rcu_preempted_readers(rnp))
		retval |= RCU_OFL_TASKS_NORM_GP;
	if (rcu_preempted_readers_exp(rnp))
		retval |= RCU_OFL_TASKS_EXP_GP;
	for (i = 0; i < 4; i++) {
		lp = &rnp->blocked_tasks[i];
		lp_root = &rnp_root->blocked_tasks[i];
		while (!list_empty(lp)) {
			tp = list_entry(lp->next, typeof(*tp), rcu_node_entry);
			raw_spin_lock(&rnp_root->lock); /* irqs already disabled */
			list_del(&tp->rcu_node_entry);
			tp->rcu_blocked_node = rnp_root;
			list_add(&tp->rcu_node_entry, lp_root);
			raw_spin_unlock(&rnp_root->lock); /* irqs remain disabled */
		}
	}
	return retval;
}

static void rcu_preempt_offline_cpu(int cpu)
{
	__rcu_offline_cpu(cpu, &rcu_preempt_state);
}

#endif /* #ifdef CONFIG_HOTPLUG_CPU */

static void rcu_preempt_check_callbacks(int cpu)
{
	struct task_struct *t = current;

	if (t->rcu_read_lock_nesting == 0) {
		rcu_preempt_qs(cpu);
		return;
	}
	if (per_cpu(rcu_preempt_data, cpu).qs_pending)
		t->rcu_read_unlock_special |= RCU_READ_UNLOCK_NEED_QS;
}

static void rcu_preempt_process_callbacks(void)
{
	__rcu_process_callbacks(&rcu_preempt_state,
				&__get_cpu_var(rcu_preempt_data));
}

void call_rcu(struct rcu_head *head, void (*func)(struct rcu_head *rcu))
{
	__call_rcu(head, func, &rcu_preempt_state);
}
EXPORT_SYMBOL_GPL(call_rcu);

void synchronize_rcu(void)
{
	struct rcu_synchronize rcu;

	if (!rcu_scheduler_active)
		return;

	init_rcu_head_on_stack(&rcu.head);
	init_completion(&rcu.completion);
	/* Will wake me after RCU finished. */
	call_rcu(&rcu.head, wakeme_after_rcu);
	/* Wait for it. */
	wait_for_completion(&rcu.completion);
	destroy_rcu_head_on_stack(&rcu.head);
}
EXPORT_SYMBOL_GPL(synchronize_rcu);

static DECLARE_WAIT_QUEUE_HEAD(sync_rcu_preempt_exp_wq);
static long sync_rcu_preempt_exp_count;
static DEFINE_MUTEX(sync_rcu_preempt_exp_mutex);

static int rcu_preempted_readers_exp(struct rcu_node *rnp)
{
	return !list_empty(&rnp->blocked_tasks[2]) ||
	       !list_empty(&rnp->blocked_tasks[3]);
}

static int sync_rcu_preempt_exp_done(struct rcu_node *rnp)
{
	return !rcu_preempted_readers_exp(rnp) &&
	       ACCESS_ONCE(rnp->expmask) == 0;
}

static void rcu_report_exp_rnp(struct rcu_state *rsp, struct rcu_node *rnp)
{
	unsigned long flags;
	unsigned long mask;

	raw_spin_lock_irqsave(&rnp->lock, flags);
	for (;;) {
		if (!sync_rcu_preempt_exp_done(rnp))
			break;
		if (rnp->parent == NULL) {
			wake_up(&sync_rcu_preempt_exp_wq);
			break;
		}
		mask = rnp->grpmask;
		raw_spin_unlock(&rnp->lock); /* irqs remain disabled */
		rnp = rnp->parent;
		raw_spin_lock(&rnp->lock); /* irqs already disabled */
		rnp->expmask &= ~mask;
	}
	raw_spin_unlock_irqrestore(&rnp->lock, flags);
}

static void
sync_rcu_preempt_exp_init(struct rcu_state *rsp, struct rcu_node *rnp)
{
	int must_wait;

	raw_spin_lock(&rnp->lock); /* irqs already disabled */
	list_splice_init(&rnp->blocked_tasks[0], &rnp->blocked_tasks[2]);
	list_splice_init(&rnp->blocked_tasks[1], &rnp->blocked_tasks[3]);
	must_wait = rcu_preempted_readers_exp(rnp);
	raw_spin_unlock(&rnp->lock); /* irqs remain disabled */
	if (!must_wait)
		rcu_report_exp_rnp(rsp, rnp);
}

void synchronize_rcu_expedited(void)
{
	unsigned long flags;
	struct rcu_node *rnp;
	struct rcu_state *rsp = &rcu_preempt_state;
	long snap;
	int trycount = 0;

	smp_mb(); /* Caller's modifications seen first by other CPUs. */
	snap = ACCESS_ONCE(sync_rcu_preempt_exp_count) + 1;
	smp_mb(); /* Above access cannot bleed into critical section. */

	/*
	 * Acquire lock, falling back to synchronize_rcu() if too many
	 * lock-acquisition failures.  Of course, if someone does the
	 * expedited grace period for us, just leave.
	 */
	while (!mutex_trylock(&sync_rcu_preempt_exp_mutex)) {
		if (trycount++ < 10)
			udelay(trycount * num_online_cpus());
		else {
			synchronize_rcu();
			return;
		}
		if ((ACCESS_ONCE(sync_rcu_preempt_exp_count) - snap) > 0)
			goto mb_ret; /* Others did our work for us. */
	}
	if ((ACCESS_ONCE(sync_rcu_preempt_exp_count) - snap) > 0)
		goto unlock_mb_ret; /* Others did our work for us. */

	/* force all RCU readers onto blocked_tasks[]. */
	synchronize_sched_expedited();

	raw_spin_lock_irqsave(&rsp->onofflock, flags);

	/* Initialize ->expmask for all non-leaf rcu_node structures. */
	rcu_for_each_nonleaf_node_breadth_first(rsp, rnp) {
		raw_spin_lock(&rnp->lock); /* irqs already disabled. */
		rnp->expmask = rnp->qsmaskinit;
		raw_spin_unlock(&rnp->lock); /* irqs remain disabled. */
	}

	/* Snapshot current state of ->blocked_tasks[] lists. */
	rcu_for_each_leaf_node(rsp, rnp)
		sync_rcu_preempt_exp_init(rsp, rnp);
	if (NUM_RCU_NODES > 1)
		sync_rcu_preempt_exp_init(rsp, rcu_get_root(rsp));

	raw_spin_unlock_irqrestore(&rsp->onofflock, flags);

	/* Wait for snapshotted ->blocked_tasks[] lists to drain. */
	rnp = rcu_get_root(rsp);
	wait_event(sync_rcu_preempt_exp_wq,
		   sync_rcu_preempt_exp_done(rnp));

	/* Clean up and exit. */
	smp_mb(); /* ensure expedited GP seen before counter increment. */
	ACCESS_ONCE(sync_rcu_preempt_exp_count)++;
unlock_mb_ret:
	mutex_unlock(&sync_rcu_preempt_exp_mutex);
mb_ret:
	smp_mb(); /* ensure subsequent action seen after grace period. */
}
EXPORT_SYMBOL_GPL(synchronize_rcu_expedited);

static int rcu_preempt_pending(int cpu)
{
	return __rcu_pending(&rcu_preempt_state,
			     &per_cpu(rcu_preempt_data, cpu));
}

static int rcu_preempt_needs_cpu(int cpu)
{
	return !!per_cpu(rcu_preempt_data, cpu).nxtlist;
}

void rcu_barrier(void)
{
	_rcu_barrier(&rcu_preempt_state, call_rcu);
}
EXPORT_SYMBOL_GPL(rcu_barrier);

static void __cpuinit rcu_preempt_init_percpu_data(int cpu)
{
	rcu_init_percpu_data(cpu, &rcu_preempt_state, 1);
}

static void rcu_preempt_send_cbs_to_orphanage(void)
{
	rcu_send_cbs_to_orphanage(&rcu_preempt_state);
}

static void __init __rcu_init_preempt(void)
{
	RCU_INIT_FLAVOR(&rcu_preempt_state, rcu_preempt_data);
}

void exit_rcu(void)
{
	struct task_struct *t = current;

	if (t->rcu_read_lock_nesting == 0)
		return;
	t->rcu_read_lock_nesting = 1;
	rcu_read_unlock();
}

#else /* #ifdef CONFIG_TREE_PREEMPT_RCU */

static void __init rcu_bootup_announce(void)
{
	printk(KERN_INFO "Hierarchical RCU implementation.\n");
	rcu_bootup_announce_oddness();
}

long rcu_batches_completed(void)
{
	return rcu_batches_completed_sched();
}
EXPORT_SYMBOL_GPL(rcu_batches_completed);

void rcu_force_quiescent_state(void)
{
	rcu_sched_force_quiescent_state();
}
EXPORT_SYMBOL_GPL(rcu_force_quiescent_state);

static void rcu_preempt_note_context_switch(int cpu)
{
}

static int rcu_preempted_readers(struct rcu_node *rnp)
{
	return 0;
}

#ifdef CONFIG_HOTPLUG_CPU

/* Because preemptible RCU does not exist, no quieting of tasks. */
static void rcu_report_unblock_qs_rnp(struct rcu_node *rnp, unsigned long flags)
{
	raw_spin_unlock_irqrestore(&rnp->lock, flags);
}

#endif /* #ifdef CONFIG_HOTPLUG_CPU */

#ifdef CONFIG_RCU_CPU_STALL_DETECTOR

static void rcu_print_detail_task_stall(struct rcu_state *rsp)
{
}

static void rcu_print_task_stall(struct rcu_node *rnp)
{
}

#endif /* #ifdef CONFIG_RCU_CPU_STALL_DETECTOR */

static void rcu_preempt_check_blocked_tasks(struct rcu_node *rnp)
{
	WARN_ON_ONCE(rnp->qsmask);
}

#ifdef CONFIG_HOTPLUG_CPU

static int rcu_preempt_offline_tasks(struct rcu_state *rsp,
				     struct rcu_node *rnp,
				     struct rcu_data *rdp)
{
	return 0;
}

static void rcu_preempt_offline_cpu(int cpu)
{
}

#endif /* #ifdef CONFIG_HOTPLUG_CPU */

static void rcu_preempt_check_callbacks(int cpu)
{
}

static void rcu_preempt_process_callbacks(void)
{
}

void call_rcu(struct rcu_head *head, void (*func)(struct rcu_head *rcu))
{
	call_rcu_sched(head, func);
}
EXPORT_SYMBOL_GPL(call_rcu);

void synchronize_rcu_expedited(void)
{
	synchronize_sched_expedited();
}
EXPORT_SYMBOL_GPL(synchronize_rcu_expedited);

#ifdef CONFIG_HOTPLUG_CPU

static void rcu_report_exp_rnp(struct rcu_state *rsp, struct rcu_node *rnp)
{
	return;
}

#endif /* #ifdef CONFIG_HOTPLUG_CPU */

static int rcu_preempt_pending(int cpu)
{
	return 0;
}

static int rcu_preempt_needs_cpu(int cpu)
{
	return 0;
}

void rcu_barrier(void)
{
	rcu_barrier_sched();
}
EXPORT_SYMBOL_GPL(rcu_barrier);

static void __cpuinit rcu_preempt_init_percpu_data(int cpu)
{
}

static void rcu_preempt_send_cbs_to_orphanage(void)
{
}

static void __init __rcu_init_preempt(void)
{
}

#endif /* #else #ifdef CONFIG_TREE_PREEMPT_RCU */

#if !defined(CONFIG_RCU_FAST_NO_HZ)

int rcu_needs_cpu(int cpu)
{
	return rcu_needs_cpu_quick_check(cpu);
}

static void rcu_needs_cpu_flush(void)
{
}

#else /* #if !defined(CONFIG_RCU_FAST_NO_HZ) */

#define RCU_NEEDS_CPU_FLUSHES 5
static DEFINE_PER_CPU(int, rcu_dyntick_drain);
static DEFINE_PER_CPU(unsigned long, rcu_dyntick_holdoff);

int rcu_needs_cpu(int cpu)
{
	int c = 0;
	int snap;
	int snap_nmi;
	int thatcpu;

	/* Check for being in the holdoff period. */
	if (per_cpu(rcu_dyntick_holdoff, cpu) == jiffies)
		return rcu_needs_cpu_quick_check(cpu);

	/* Don't bother unless we are the last non-dyntick-idle CPU. */
	for_each_online_cpu(thatcpu) {
		if (thatcpu == cpu)
			continue;
		snap = per_cpu(rcu_dynticks, thatcpu).dynticks;
		snap_nmi = per_cpu(rcu_dynticks, thatcpu).dynticks_nmi;
		smp_mb(); /* Order sampling of snap with end of grace period. */
		if (((snap & 0x1) != 0) || ((snap_nmi & 0x1) != 0)) {
			per_cpu(rcu_dyntick_drain, cpu) = 0;
			per_cpu(rcu_dyntick_holdoff, cpu) = jiffies - 1;
			return rcu_needs_cpu_quick_check(cpu);
		}
	}

	/* Check and update the rcu_dyntick_drain sequencing. */
	if (per_cpu(rcu_dyntick_drain, cpu) <= 0) {
		/* First time through, initialize the counter. */
		per_cpu(rcu_dyntick_drain, cpu) = RCU_NEEDS_CPU_FLUSHES;
	} else if (--per_cpu(rcu_dyntick_drain, cpu) <= 0) {
		/* We have hit the limit, so time to give up. */
		per_cpu(rcu_dyntick_holdoff, cpu) = jiffies;
		return rcu_needs_cpu_quick_check(cpu);
	}

	/* Do one step pushing remaining RCU callbacks through. */
	if (per_cpu(rcu_sched_data, cpu).nxtlist) {
		rcu_sched_qs(cpu);
		force_quiescent_state(&rcu_sched_state, 0);
		c = c || per_cpu(rcu_sched_data, cpu).nxtlist;
	}
	if (per_cpu(rcu_bh_data, cpu).nxtlist) {
		rcu_bh_qs(cpu);
		force_quiescent_state(&rcu_bh_state, 0);
		c = c || per_cpu(rcu_bh_data, cpu).nxtlist;
	}

	/* If RCU callbacks are still pending, RCU still needs this CPU. */
	if (c)
		raise_softirq(RCU_SOFTIRQ);
	return c;
}

static void rcu_needs_cpu_flush(void)
{
	int cpu = smp_processor_id();
	unsigned long flags;

	if (per_cpu(rcu_dyntick_drain, cpu) <= 0)
		return;
	local_irq_save(flags);
	(void)rcu_needs_cpu(cpu);
	local_irq_restore(flags);
}

#endif /* #else #if !defined(CONFIG_RCU_FAST_NO_HZ) */
