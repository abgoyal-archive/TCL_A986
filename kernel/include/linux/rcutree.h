

#ifndef __LINUX_RCUTREE_H
#define __LINUX_RCUTREE_H

struct notifier_block;

extern void rcu_sched_qs(int cpu);
extern void rcu_bh_qs(int cpu);
extern void rcu_note_context_switch(int cpu);
extern int rcu_needs_cpu(int cpu);

#ifdef CONFIG_TREE_PREEMPT_RCU

extern void __rcu_read_lock(void);
extern void __rcu_read_unlock(void);
extern void synchronize_rcu(void);
extern void exit_rcu(void);

#define rcu_preempt_depth() (current->rcu_read_lock_nesting)

#else /* #ifdef CONFIG_TREE_PREEMPT_RCU */

static inline void __rcu_read_lock(void)
{
	preempt_disable();
}

static inline void __rcu_read_unlock(void)
{
	preempt_enable();
}

#define synchronize_rcu synchronize_sched

static inline void exit_rcu(void)
{
}

static inline int rcu_preempt_depth(void)
{
	return 0;
}

#endif /* #else #ifdef CONFIG_TREE_PREEMPT_RCU */

static inline void __rcu_read_lock_bh(void)
{
	local_bh_disable();
}
static inline void __rcu_read_unlock_bh(void)
{
	local_bh_enable();
}

extern void call_rcu_sched(struct rcu_head *head,
			   void (*func)(struct rcu_head *rcu));
extern void synchronize_rcu_bh(void);
extern void synchronize_sched(void);
extern void synchronize_rcu_expedited(void);

static inline void synchronize_rcu_bh_expedited(void)
{
	synchronize_sched_expedited();
}

extern void rcu_check_callbacks(int cpu, int user);

extern long rcu_batches_completed(void);
extern long rcu_batches_completed_bh(void);
extern long rcu_batches_completed_sched(void);
extern void rcu_force_quiescent_state(void);
extern void rcu_bh_force_quiescent_state(void);
extern void rcu_sched_force_quiescent_state(void);

#ifdef CONFIG_NO_HZ
void rcu_enter_nohz(void);
void rcu_exit_nohz(void);
#else /* CONFIG_NO_HZ */
static inline void rcu_enter_nohz(void)
{
}
static inline void rcu_exit_nohz(void)
{
}
#endif /* CONFIG_NO_HZ */

/* A context switch is a grace period for RCU-sched and RCU-bh. */
static inline int rcu_blocking_is_gp(void)
{
	return num_online_cpus() == 1;
}

extern void rcu_scheduler_starting(void);
extern int rcu_scheduler_active __read_mostly;

#endif /* __LINUX_RCUTREE_H */
