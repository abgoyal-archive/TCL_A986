

#ifndef __LINUX_RCUPDATE_H
#define __LINUX_RCUPDATE_H

#include <linux/cache.h>
#include <linux/spinlock.h>
#include <linux/threads.h>
#include <linux/cpumask.h>
#include <linux/seqlock.h>
#include <linux/lockdep.h>
#include <linux/completion.h>

#ifdef CONFIG_RCU_TORTURE_TEST
extern int rcutorture_runnable; /* for sysctl */
#endif /* #ifdef CONFIG_RCU_TORTURE_TEST */

struct rcu_head {
	struct rcu_head *next;
	void (*func)(struct rcu_head *head);
};

/* Exported common interfaces */
extern void rcu_barrier(void);
extern void rcu_barrier_bh(void);
extern void rcu_barrier_sched(void);
extern void synchronize_sched_expedited(void);
extern int sched_expedited_torture_stats(char *page);

/* Internal to kernel */
extern void rcu_init(void);

#if defined(CONFIG_TREE_RCU) || defined(CONFIG_TREE_PREEMPT_RCU)
#include <linux/rcutree.h>
#elif defined(CONFIG_TINY_RCU)
#include <linux/rcutiny.h>
#else
#error "Unknown RCU implementation specified to kernel configuration"
#endif

#define RCU_HEAD_INIT	{ .next = NULL, .func = NULL }
#define RCU_HEAD(head) struct rcu_head head = RCU_HEAD_INIT
#define INIT_RCU_HEAD(ptr) do { \
       (ptr)->next = NULL; (ptr)->func = NULL; \
} while (0)

static inline void init_rcu_head_on_stack(struct rcu_head *head)
{
}

static inline void destroy_rcu_head_on_stack(struct rcu_head *head)
{
}

#ifdef CONFIG_DEBUG_LOCK_ALLOC

extern struct lockdep_map rcu_lock_map;
# define rcu_read_acquire() \
		lock_acquire(&rcu_lock_map, 0, 0, 2, 1, NULL, _THIS_IP_)
# define rcu_read_release()	lock_release(&rcu_lock_map, 1, _THIS_IP_)

extern struct lockdep_map rcu_bh_lock_map;
# define rcu_read_acquire_bh() \
		lock_acquire(&rcu_bh_lock_map, 0, 0, 2, 1, NULL, _THIS_IP_)
# define rcu_read_release_bh()	lock_release(&rcu_bh_lock_map, 1, _THIS_IP_)

extern struct lockdep_map rcu_sched_lock_map;
# define rcu_read_acquire_sched() \
		lock_acquire(&rcu_sched_lock_map, 0, 0, 2, 1, NULL, _THIS_IP_)
# define rcu_read_release_sched() \
		lock_release(&rcu_sched_lock_map, 1, _THIS_IP_)

extern int debug_lockdep_rcu_enabled(void);

static inline int rcu_read_lock_held(void)
{
	if (!debug_lockdep_rcu_enabled())
		return 1;
	return lock_is_held(&rcu_lock_map);
}

extern int rcu_read_lock_bh_held(void);

#ifdef CONFIG_PREEMPT
static inline int rcu_read_lock_sched_held(void)
{
	int lockdep_opinion = 0;

	if (!debug_lockdep_rcu_enabled())
		return 1;
	if (debug_locks)
		lockdep_opinion = lock_is_held(&rcu_sched_lock_map);
	return lockdep_opinion || preempt_count() != 0 || irqs_disabled();
}
#else /* #ifdef CONFIG_PREEMPT */
static inline int rcu_read_lock_sched_held(void)
{
	return 1;
}
#endif /* #else #ifdef CONFIG_PREEMPT */

#else /* #ifdef CONFIG_DEBUG_LOCK_ALLOC */

# define rcu_read_acquire()		do { } while (0)
# define rcu_read_release()		do { } while (0)
# define rcu_read_acquire_bh()		do { } while (0)
# define rcu_read_release_bh()		do { } while (0)
# define rcu_read_acquire_sched()	do { } while (0)
# define rcu_read_release_sched()	do { } while (0)

static inline int rcu_read_lock_held(void)
{
	return 1;
}

static inline int rcu_read_lock_bh_held(void)
{
	return 1;
}

#ifdef CONFIG_PREEMPT
static inline int rcu_read_lock_sched_held(void)
{
	return preempt_count() != 0 || irqs_disabled();
}
#else /* #ifdef CONFIG_PREEMPT */
static inline int rcu_read_lock_sched_held(void)
{
	return 1;
}
#endif /* #else #ifdef CONFIG_PREEMPT */

#endif /* #else #ifdef CONFIG_DEBUG_LOCK_ALLOC */

#ifdef CONFIG_PROVE_RCU

extern int rcu_my_thread_group_empty(void);

#define __do_rcu_dereference_check(c)					\
	do {								\
		static bool __warned;					\
		if (debug_lockdep_rcu_enabled() && !__warned && !(c)) {	\
			__warned = true;				\
			lockdep_rcu_dereference(__FILE__, __LINE__);	\
		}							\
	} while (0)

#define rcu_dereference_check(p, c) \
	({ \
		__do_rcu_dereference_check(c); \
		rcu_dereference_raw(p); \
	})

#define rcu_dereference_protected(p, c) \
	({ \
		__do_rcu_dereference_check(c); \
		(p); \
	})

#else /* #ifdef CONFIG_PROVE_RCU */

#define rcu_dereference_check(p, c)	rcu_dereference_raw(p)
#define rcu_dereference_protected(p, c) (p)

#endif /* #else #ifdef CONFIG_PROVE_RCU */

#define rcu_access_pointer(p)	ACCESS_ONCE(p)

static inline void rcu_read_lock(void)
{
	__rcu_read_lock();
	__acquire(RCU);
	rcu_read_acquire();
}


static inline void rcu_read_unlock(void)
{
	rcu_read_release();
	__release(RCU);
	__rcu_read_unlock();
}

static inline void rcu_read_lock_bh(void)
{
	__rcu_read_lock_bh();
	__acquire(RCU_BH);
	rcu_read_acquire_bh();
}

static inline void rcu_read_unlock_bh(void)
{
	rcu_read_release_bh();
	__release(RCU_BH);
	__rcu_read_unlock_bh();
}

static inline void rcu_read_lock_sched(void)
{
	preempt_disable();
	__acquire(RCU_SCHED);
	rcu_read_acquire_sched();
}

/* Used by lockdep and tracing: cannot be traced, cannot call lockdep. */
static inline notrace void rcu_read_lock_sched_notrace(void)
{
	preempt_disable_notrace();
	__acquire(RCU_SCHED);
}

static inline void rcu_read_unlock_sched(void)
{
	rcu_read_release_sched();
	__release(RCU_SCHED);
	preempt_enable();
}

/* Used by lockdep and tracing: cannot be traced, cannot call lockdep. */
static inline notrace void rcu_read_unlock_sched_notrace(void)
{
	__release(RCU_SCHED);
	preempt_enable_notrace();
}


#define rcu_dereference_raw(p)	({ \
				typeof(p) _________p1 = ACCESS_ONCE(p); \
				smp_read_barrier_depends(); \
				(_________p1); \
				})

#define rcu_dereference(p) \
	rcu_dereference_check(p, rcu_read_lock_held())

#define rcu_dereference_bh(p) \
		rcu_dereference_check(p, rcu_read_lock_bh_held())

#define rcu_dereference_sched(p) \
		rcu_dereference_check(p, rcu_read_lock_sched_held())


#define rcu_assign_pointer(p, v) \
	({ \
		if (!__builtin_constant_p(v) || \
		    ((v) != NULL)) \
			smp_wmb(); \
		(p) = (v); \
	})

/* Infrastructure to implement the synchronize_() primitives. */

struct rcu_synchronize {
	struct rcu_head head;
	struct completion completion;
};

extern void wakeme_after_rcu(struct rcu_head  *head);

extern void call_rcu(struct rcu_head *head,
			      void (*func)(struct rcu_head *head));

extern void call_rcu_bh(struct rcu_head *head,
			void (*func)(struct rcu_head *head));

#endif /* __LINUX_RCUPDATE_H */
