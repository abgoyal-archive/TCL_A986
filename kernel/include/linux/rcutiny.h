
#ifndef __LINUX_TINY_H
#define __LINUX_TINY_H

#include <linux/cache.h>

void rcu_sched_qs(int cpu);
void rcu_bh_qs(int cpu);
static inline void rcu_note_context_switch(int cpu)
{
	rcu_sched_qs(cpu);
}

#define __rcu_read_lock()	preempt_disable()
#define __rcu_read_unlock()	preempt_enable()
#define __rcu_read_lock_bh()	local_bh_disable()
#define __rcu_read_unlock_bh()	local_bh_enable()
#define call_rcu_sched		call_rcu

#define rcu_init_sched()	do { } while (0)
extern void rcu_check_callbacks(int cpu, int user);

static inline int rcu_needs_cpu(int cpu)
{
	return 0;
}

static inline long rcu_batches_completed(void)
{
	return 0;
}

static inline long rcu_batches_completed_bh(void)
{
	return 0;
}

static inline void rcu_force_quiescent_state(void)
{
}

static inline void rcu_bh_force_quiescent_state(void)
{
}

static inline void rcu_sched_force_quiescent_state(void)
{
}

extern void synchronize_sched(void);

static inline void synchronize_rcu(void)
{
	synchronize_sched();
}

static inline void synchronize_rcu_bh(void)
{
	synchronize_sched();
}

static inline void synchronize_rcu_expedited(void)
{
	synchronize_sched();
}

static inline void synchronize_rcu_bh_expedited(void)
{
	synchronize_sched();
}

struct notifier_block;

#ifdef CONFIG_NO_HZ

extern void rcu_enter_nohz(void);
extern void rcu_exit_nohz(void);

#else /* #ifdef CONFIG_NO_HZ */

static inline void rcu_enter_nohz(void)
{
}

static inline void rcu_exit_nohz(void)
{
}

#endif /* #else #ifdef CONFIG_NO_HZ */

static inline void exit_rcu(void)
{
}

static inline int rcu_preempt_depth(void)
{
	return 0;
}

#ifdef CONFIG_DEBUG_LOCK_ALLOC

extern int rcu_scheduler_active __read_mostly;
extern void rcu_scheduler_starting(void);

#else /* #ifdef CONFIG_DEBUG_LOCK_ALLOC */

static inline void rcu_scheduler_starting(void)
{
}

#endif /* #else #ifdef CONFIG_DEBUG_LOCK_ALLOC */

#endif /* __LINUX_RCUTINY_H */
