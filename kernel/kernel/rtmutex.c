
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/timer.h>

#include "rtmutex_common.h"


static void
rt_mutex_set_owner(struct rt_mutex *lock, struct task_struct *owner,
		   unsigned long mask)
{
	unsigned long val = (unsigned long)owner | mask;

	if (rt_mutex_has_waiters(lock))
		val |= RT_MUTEX_HAS_WAITERS;

	lock->owner = (struct task_struct *)val;
}

static inline void clear_rt_mutex_waiters(struct rt_mutex *lock)
{
	lock->owner = (struct task_struct *)
			((unsigned long)lock->owner & ~RT_MUTEX_HAS_WAITERS);
}

static void fixup_rt_mutex_waiters(struct rt_mutex *lock)
{
	if (!rt_mutex_has_waiters(lock))
		clear_rt_mutex_waiters(lock);
}

#if defined(__HAVE_ARCH_CMPXCHG) && !defined(CONFIG_DEBUG_RT_MUTEXES)
# define rt_mutex_cmpxchg(l,c,n)	(cmpxchg(&l->owner, c, n) == c)
static inline void mark_rt_mutex_waiters(struct rt_mutex *lock)
{
	unsigned long owner, *p = (unsigned long *) &lock->owner;

	do {
		owner = *p;
	} while (cmpxchg(p, owner, owner | RT_MUTEX_HAS_WAITERS) != owner);
}
#else
# define rt_mutex_cmpxchg(l,c,n)	(0)
static inline void mark_rt_mutex_waiters(struct rt_mutex *lock)
{
	lock->owner = (struct task_struct *)
			((unsigned long)lock->owner | RT_MUTEX_HAS_WAITERS);
}
#endif

int rt_mutex_getprio(struct task_struct *task)
{
	if (likely(!task_has_pi_waiters(task)))
		return task->normal_prio;

	return min(task_top_pi_waiter(task)->pi_list_entry.prio,
		   task->normal_prio);
}

static void __rt_mutex_adjust_prio(struct task_struct *task)
{
	int prio = rt_mutex_getprio(task);

	if (task->prio != prio)
		rt_mutex_setprio(task, prio);
}

static void rt_mutex_adjust_prio(struct task_struct *task)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&task->pi_lock, flags);
	__rt_mutex_adjust_prio(task);
	raw_spin_unlock_irqrestore(&task->pi_lock, flags);
}

int max_lock_depth = 1024;

static int rt_mutex_adjust_prio_chain(struct task_struct *task,
				      int deadlock_detect,
				      struct rt_mutex *orig_lock,
				      struct rt_mutex_waiter *orig_waiter,
				      struct task_struct *top_task)
{
	struct rt_mutex *lock;
	struct rt_mutex_waiter *waiter, *top_waiter = orig_waiter;
	int detect_deadlock, ret = 0, depth = 0;
	unsigned long flags;

	detect_deadlock = debug_rt_mutex_detect_deadlock(orig_waiter,
							 deadlock_detect);

	/*
	 * The (de)boosting is a step by step approach with a lot of
	 * pitfalls. We want this to be preemptible and we want hold a
	 * maximum of two locks per step. So we have to check
	 * carefully whether things change under us.
	 */
 again:
	if (++depth > max_lock_depth) {
		static int prev_max;

		/*
		 * Print this only once. If the admin changes the limit,
		 * print a new message when reaching the limit again.
		 */
		if (prev_max != max_lock_depth) {
			prev_max = max_lock_depth;
			printk(KERN_WARNING "Maximum lock depth %d reached "
			       "task: %s (%d)\n", max_lock_depth,
			       top_task->comm, task_pid_nr(top_task));
		}
		put_task_struct(task);

		return deadlock_detect ? -EDEADLK : 0;
	}
 retry:
	/*
	 * Task can not go away as we did a get_task() before !
	 */
	raw_spin_lock_irqsave(&task->pi_lock, flags);

	waiter = task->pi_blocked_on;
	/*
	 * Check whether the end of the boosting chain has been
	 * reached or the state of the chain has changed while we
	 * dropped the locks.
	 */
	if (!waiter || !waiter->task)
		goto out_unlock_pi;

	/*
	 * Check the orig_waiter state. After we dropped the locks,
	 * the previous owner of the lock might have released the lock
	 * and made us the pending owner:
	 */
	if (orig_waiter && !orig_waiter->task)
		goto out_unlock_pi;

	/*
	 * Drop out, when the task has no waiters. Note,
	 * top_waiter can be NULL, when we are in the deboosting
	 * mode!
	 */
	if (top_waiter && (!task_has_pi_waiters(task) ||
			   top_waiter != task_top_pi_waiter(task)))
		goto out_unlock_pi;

	/*
	 * When deadlock detection is off then we check, if further
	 * priority adjustment is necessary.
	 */
	if (!detect_deadlock && waiter->list_entry.prio == task->prio)
		goto out_unlock_pi;

	lock = waiter->lock;
	if (!raw_spin_trylock(&lock->wait_lock)) {
		raw_spin_unlock_irqrestore(&task->pi_lock, flags);
		cpu_relax();
		goto retry;
	}

	/* Deadlock detection */
	if (lock == orig_lock || rt_mutex_owner(lock) == top_task) {
		debug_rt_mutex_deadlock(deadlock_detect, orig_waiter, lock);
		raw_spin_unlock(&lock->wait_lock);
		ret = deadlock_detect ? -EDEADLK : 0;
		goto out_unlock_pi;
	}

	top_waiter = rt_mutex_top_waiter(lock);

	/* Requeue the waiter */
	plist_del(&waiter->list_entry, &lock->wait_list);
	waiter->list_entry.prio = task->prio;
	plist_add(&waiter->list_entry, &lock->wait_list);

	/* Release the task */
	raw_spin_unlock_irqrestore(&task->pi_lock, flags);
	put_task_struct(task);

	/* Grab the next task */
	task = rt_mutex_owner(lock);
	get_task_struct(task);
	raw_spin_lock_irqsave(&task->pi_lock, flags);

	if (waiter == rt_mutex_top_waiter(lock)) {
		/* Boost the owner */
		plist_del(&top_waiter->pi_list_entry, &task->pi_waiters);
		waiter->pi_list_entry.prio = waiter->list_entry.prio;
		plist_add(&waiter->pi_list_entry, &task->pi_waiters);
		__rt_mutex_adjust_prio(task);

	} else if (top_waiter == waiter) {
		/* Deboost the owner */
		plist_del(&waiter->pi_list_entry, &task->pi_waiters);
		waiter = rt_mutex_top_waiter(lock);
		waiter->pi_list_entry.prio = waiter->list_entry.prio;
		plist_add(&waiter->pi_list_entry, &task->pi_waiters);
		__rt_mutex_adjust_prio(task);
	}

	raw_spin_unlock_irqrestore(&task->pi_lock, flags);

	top_waiter = rt_mutex_top_waiter(lock);
	raw_spin_unlock(&lock->wait_lock);

	if (!detect_deadlock && waiter != top_waiter)
		goto out_put_task;

	goto again;

 out_unlock_pi:
	raw_spin_unlock_irqrestore(&task->pi_lock, flags);
 out_put_task:
	put_task_struct(task);

	return ret;
}

static inline int try_to_steal_lock(struct rt_mutex *lock,
				    struct task_struct *task)
{
	struct task_struct *pendowner = rt_mutex_owner(lock);
	struct rt_mutex_waiter *next;
	unsigned long flags;

	if (!rt_mutex_owner_pending(lock))
		return 0;

	if (pendowner == task)
		return 1;

	raw_spin_lock_irqsave(&pendowner->pi_lock, flags);
	if (task->prio >= pendowner->prio) {
		raw_spin_unlock_irqrestore(&pendowner->pi_lock, flags);
		return 0;
	}

	/*
	 * Check if a waiter is enqueued on the pending owners
	 * pi_waiters list. Remove it and readjust pending owners
	 * priority.
	 */
	if (likely(!rt_mutex_has_waiters(lock))) {
		raw_spin_unlock_irqrestore(&pendowner->pi_lock, flags);
		return 1;
	}

	/* No chain handling, pending owner is not blocked on anything: */
	next = rt_mutex_top_waiter(lock);
	plist_del(&next->pi_list_entry, &pendowner->pi_waiters);
	__rt_mutex_adjust_prio(pendowner);
	raw_spin_unlock_irqrestore(&pendowner->pi_lock, flags);

	/*
	 * We are going to steal the lock and a waiter was
	 * enqueued on the pending owners pi_waiters queue. So
	 * we have to enqueue this waiter into
	 * task->pi_waiters list. This covers the case,
	 * where task is boosted because it holds another
	 * lock and gets unboosted because the booster is
	 * interrupted, so we would delay a waiter with higher
	 * priority as task->normal_prio.
	 *
	 * Note: in the rare case of a SCHED_OTHER task changing
	 * its priority and thus stealing the lock, next->task
	 * might be task:
	 */
	if (likely(next->task != task)) {
		raw_spin_lock_irqsave(&task->pi_lock, flags);
		plist_add(&next->pi_list_entry, &task->pi_waiters);
		__rt_mutex_adjust_prio(task);
		raw_spin_unlock_irqrestore(&task->pi_lock, flags);
	}
	return 1;
}

static int try_to_take_rt_mutex(struct rt_mutex *lock)
{
	/*
	 * We have to be careful here if the atomic speedups are
	 * enabled, such that, when
	 *  - no other waiter is on the lock
	 *  - the lock has been released since we did the cmpxchg
	 * the lock can be released or taken while we are doing the
	 * checks and marking the lock with RT_MUTEX_HAS_WAITERS.
	 *
	 * The atomic acquire/release aware variant of
	 * mark_rt_mutex_waiters uses a cmpxchg loop. After setting
	 * the WAITERS bit, the atomic release / acquire can not
	 * happen anymore and lock->wait_lock protects us from the
	 * non-atomic case.
	 *
	 * Note, that this might set lock->owner =
	 * RT_MUTEX_HAS_WAITERS in the case the lock is not contended
	 * any more. This is fixed up when we take the ownership.
	 * This is the transitional state explained at the top of this file.
	 */
	mark_rt_mutex_waiters(lock);

	if (rt_mutex_owner(lock) && !try_to_steal_lock(lock, current))
		return 0;

	/* We got the lock. */
	debug_rt_mutex_lock(lock);

	rt_mutex_set_owner(lock, current, 0);

	rt_mutex_deadlock_account_lock(lock, current);

	return 1;
}

static int task_blocks_on_rt_mutex(struct rt_mutex *lock,
				   struct rt_mutex_waiter *waiter,
				   struct task_struct *task,
				   int detect_deadlock)
{
	struct task_struct *owner = rt_mutex_owner(lock);
	struct rt_mutex_waiter *top_waiter = waiter;
	unsigned long flags;
	int chain_walk = 0, res;

	raw_spin_lock_irqsave(&task->pi_lock, flags);
	__rt_mutex_adjust_prio(task);
	waiter->task = task;
	waiter->lock = lock;
	plist_node_init(&waiter->list_entry, task->prio);
	plist_node_init(&waiter->pi_list_entry, task->prio);

	/* Get the top priority waiter on the lock */
	if (rt_mutex_has_waiters(lock))
		top_waiter = rt_mutex_top_waiter(lock);
	plist_add(&waiter->list_entry, &lock->wait_list);

	task->pi_blocked_on = waiter;

	raw_spin_unlock_irqrestore(&task->pi_lock, flags);

	if (waiter == rt_mutex_top_waiter(lock)) {
		raw_spin_lock_irqsave(&owner->pi_lock, flags);
		plist_del(&top_waiter->pi_list_entry, &owner->pi_waiters);
		plist_add(&waiter->pi_list_entry, &owner->pi_waiters);

		__rt_mutex_adjust_prio(owner);
		if (owner->pi_blocked_on)
			chain_walk = 1;
		raw_spin_unlock_irqrestore(&owner->pi_lock, flags);
	}
	else if (debug_rt_mutex_detect_deadlock(waiter, detect_deadlock))
		chain_walk = 1;

	if (!chain_walk)
		return 0;

	/*
	 * The owner can't disappear while holding a lock,
	 * so the owner struct is protected by wait_lock.
	 * Gets dropped in rt_mutex_adjust_prio_chain()!
	 */
	get_task_struct(owner);

	raw_spin_unlock(&lock->wait_lock);

	res = rt_mutex_adjust_prio_chain(owner, detect_deadlock, lock, waiter,
					 task);

	raw_spin_lock(&lock->wait_lock);

	return res;
}

static void wakeup_next_waiter(struct rt_mutex *lock)
{
	struct rt_mutex_waiter *waiter;
	struct task_struct *pendowner;
	unsigned long flags;

	raw_spin_lock_irqsave(&current->pi_lock, flags);

	waiter = rt_mutex_top_waiter(lock);
	plist_del(&waiter->list_entry, &lock->wait_list);

	/*
	 * Remove it from current->pi_waiters. We do not adjust a
	 * possible priority boost right now. We execute wakeup in the
	 * boosted mode and go back to normal after releasing
	 * lock->wait_lock.
	 */
	plist_del(&waiter->pi_list_entry, &current->pi_waiters);
	pendowner = waiter->task;
	waiter->task = NULL;

	rt_mutex_set_owner(lock, pendowner, RT_MUTEX_OWNER_PENDING);

	raw_spin_unlock_irqrestore(&current->pi_lock, flags);

	/*
	 * Clear the pi_blocked_on variable and enqueue a possible
	 * waiter into the pi_waiters list of the pending owner. This
	 * prevents that in case the pending owner gets unboosted a
	 * waiter with higher priority than pending-owner->normal_prio
	 * is blocked on the unboosted (pending) owner.
	 */
	raw_spin_lock_irqsave(&pendowner->pi_lock, flags);

	WARN_ON(!pendowner->pi_blocked_on);
	WARN_ON(pendowner->pi_blocked_on != waiter);
	WARN_ON(pendowner->pi_blocked_on->lock != lock);

	pendowner->pi_blocked_on = NULL;

	if (rt_mutex_has_waiters(lock)) {
		struct rt_mutex_waiter *next;

		next = rt_mutex_top_waiter(lock);
		plist_add(&next->pi_list_entry, &pendowner->pi_waiters);
	}
	raw_spin_unlock_irqrestore(&pendowner->pi_lock, flags);

	wake_up_process(pendowner);
}

static void remove_waiter(struct rt_mutex *lock,
			  struct rt_mutex_waiter *waiter)
{
	int first = (waiter == rt_mutex_top_waiter(lock));
	struct task_struct *owner = rt_mutex_owner(lock);
	unsigned long flags;
	int chain_walk = 0;

	raw_spin_lock_irqsave(&current->pi_lock, flags);
	plist_del(&waiter->list_entry, &lock->wait_list);
	waiter->task = NULL;
	current->pi_blocked_on = NULL;
	raw_spin_unlock_irqrestore(&current->pi_lock, flags);

	if (first && owner != current) {

		raw_spin_lock_irqsave(&owner->pi_lock, flags);

		plist_del(&waiter->pi_list_entry, &owner->pi_waiters);

		if (rt_mutex_has_waiters(lock)) {
			struct rt_mutex_waiter *next;

			next = rt_mutex_top_waiter(lock);
			plist_add(&next->pi_list_entry, &owner->pi_waiters);
		}
		__rt_mutex_adjust_prio(owner);

		if (owner->pi_blocked_on)
			chain_walk = 1;

		raw_spin_unlock_irqrestore(&owner->pi_lock, flags);
	}

	WARN_ON(!plist_node_empty(&waiter->pi_list_entry));

	if (!chain_walk)
		return;

	/* gets dropped in rt_mutex_adjust_prio_chain()! */
	get_task_struct(owner);

	raw_spin_unlock(&lock->wait_lock);

	rt_mutex_adjust_prio_chain(owner, 0, lock, NULL, current);

	raw_spin_lock(&lock->wait_lock);
}

void rt_mutex_adjust_pi(struct task_struct *task)
{
	struct rt_mutex_waiter *waiter;
	unsigned long flags;

	raw_spin_lock_irqsave(&task->pi_lock, flags);

	waiter = task->pi_blocked_on;
	if (!waiter || waiter->list_entry.prio == task->prio) {
		raw_spin_unlock_irqrestore(&task->pi_lock, flags);
		return;
	}

	raw_spin_unlock_irqrestore(&task->pi_lock, flags);

	/* gets dropped in rt_mutex_adjust_prio_chain()! */
	get_task_struct(task);
	rt_mutex_adjust_prio_chain(task, 0, NULL, NULL, task);
}

static int __sched
__rt_mutex_slowlock(struct rt_mutex *lock, int state,
		    struct hrtimer_sleeper *timeout,
		    struct rt_mutex_waiter *waiter,
		    int detect_deadlock)
{
	int ret = 0;

	for (;;) {
		/* Try to acquire the lock: */
		if (try_to_take_rt_mutex(lock))
			break;

		/*
		 * TASK_INTERRUPTIBLE checks for signals and
		 * timeout. Ignored otherwise.
		 */
		if (unlikely(state == TASK_INTERRUPTIBLE)) {
			/* Signal pending? */
			if (signal_pending(current))
				ret = -EINTR;
			if (timeout && !timeout->task)
				ret = -ETIMEDOUT;
			if (ret)
				break;
		}

		/*
		 * waiter->task is NULL the first time we come here and
		 * when we have been woken up by the previous owner
		 * but the lock got stolen by a higher prio task.
		 */
		if (!waiter->task) {
			ret = task_blocks_on_rt_mutex(lock, waiter, current,
						      detect_deadlock);
			/*
			 * If we got woken up by the owner then start loop
			 * all over without going into schedule to try
			 * to get the lock now:
			 */
			if (unlikely(!waiter->task)) {
				/*
				 * Reset the return value. We might
				 * have returned with -EDEADLK and the
				 * owner released the lock while we
				 * were walking the pi chain.
				 */
				ret = 0;
				continue;
			}
			if (unlikely(ret))
				break;
		}

		raw_spin_unlock(&lock->wait_lock);

		debug_rt_mutex_print_deadlock(waiter);

		if (waiter->task)
			schedule_rt_mutex(lock);

		raw_spin_lock(&lock->wait_lock);
		set_current_state(state);
	}

	return ret;
}

static int __sched
rt_mutex_slowlock(struct rt_mutex *lock, int state,
		  struct hrtimer_sleeper *timeout,
		  int detect_deadlock)
{
	struct rt_mutex_waiter waiter;
	int ret = 0;

	debug_rt_mutex_init_waiter(&waiter);
	waiter.task = NULL;

	raw_spin_lock(&lock->wait_lock);

	/* Try to acquire the lock again: */
	if (try_to_take_rt_mutex(lock)) {
		raw_spin_unlock(&lock->wait_lock);
		return 0;
	}

	set_current_state(state);

	/* Setup the timer, when timeout != NULL */
	if (unlikely(timeout)) {
		hrtimer_start_expires(&timeout->timer, HRTIMER_MODE_ABS);
		if (!hrtimer_active(&timeout->timer))
			timeout->task = NULL;
	}

	ret = __rt_mutex_slowlock(lock, state, timeout, &waiter,
				  detect_deadlock);

	set_current_state(TASK_RUNNING);

	if (unlikely(waiter.task))
		remove_waiter(lock, &waiter);

	/*
	 * try_to_take_rt_mutex() sets the waiter bit
	 * unconditionally. We might have to fix that up.
	 */
	fixup_rt_mutex_waiters(lock);

	raw_spin_unlock(&lock->wait_lock);

	/* Remove pending timer: */
	if (unlikely(timeout))
		hrtimer_cancel(&timeout->timer);

	/*
	 * Readjust priority, when we did not get the lock. We might
	 * have been the pending owner and boosted. Since we did not
	 * take the lock, the PI boost has to go.
	 */
	if (unlikely(ret))
		rt_mutex_adjust_prio(current);

	debug_rt_mutex_free_waiter(&waiter);

	return ret;
}

static inline int
rt_mutex_slowtrylock(struct rt_mutex *lock)
{
	int ret = 0;

	raw_spin_lock(&lock->wait_lock);

	if (likely(rt_mutex_owner(lock) != current)) {

		ret = try_to_take_rt_mutex(lock);
		/*
		 * try_to_take_rt_mutex() sets the lock waiters
		 * bit unconditionally. Clean this up.
		 */
		fixup_rt_mutex_waiters(lock);
	}

	raw_spin_unlock(&lock->wait_lock);

	return ret;
}

static void __sched
rt_mutex_slowunlock(struct rt_mutex *lock)
{
	raw_spin_lock(&lock->wait_lock);

	debug_rt_mutex_unlock(lock);

	rt_mutex_deadlock_account_unlock(current);

	if (!rt_mutex_has_waiters(lock)) {
		lock->owner = NULL;
		raw_spin_unlock(&lock->wait_lock);
		return;
	}

	wakeup_next_waiter(lock);

	raw_spin_unlock(&lock->wait_lock);

	/* Undo pi boosting if necessary: */
	rt_mutex_adjust_prio(current);
}

static inline int
rt_mutex_fastlock(struct rt_mutex *lock, int state,
		  int detect_deadlock,
		  int (*slowfn)(struct rt_mutex *lock, int state,
				struct hrtimer_sleeper *timeout,
				int detect_deadlock))
{
	if (!detect_deadlock && likely(rt_mutex_cmpxchg(lock, NULL, current))) {
		rt_mutex_deadlock_account_lock(lock, current);
		return 0;
	} else
		return slowfn(lock, state, NULL, detect_deadlock);
}

static inline int
rt_mutex_timed_fastlock(struct rt_mutex *lock, int state,
			struct hrtimer_sleeper *timeout, int detect_deadlock,
			int (*slowfn)(struct rt_mutex *lock, int state,
				      struct hrtimer_sleeper *timeout,
				      int detect_deadlock))
{
	if (!detect_deadlock && likely(rt_mutex_cmpxchg(lock, NULL, current))) {
		rt_mutex_deadlock_account_lock(lock, current);
		return 0;
	} else
		return slowfn(lock, state, timeout, detect_deadlock);
}

static inline int
rt_mutex_fasttrylock(struct rt_mutex *lock,
		     int (*slowfn)(struct rt_mutex *lock))
{
	if (likely(rt_mutex_cmpxchg(lock, NULL, current))) {
		rt_mutex_deadlock_account_lock(lock, current);
		return 1;
	}
	return slowfn(lock);
}

static inline void
rt_mutex_fastunlock(struct rt_mutex *lock,
		    void (*slowfn)(struct rt_mutex *lock))
{
	if (likely(rt_mutex_cmpxchg(lock, current, NULL)))
		rt_mutex_deadlock_account_unlock(current);
	else
		slowfn(lock);
}

void __sched rt_mutex_lock(struct rt_mutex *lock)
{
	might_sleep();

	rt_mutex_fastlock(lock, TASK_UNINTERRUPTIBLE, 0, rt_mutex_slowlock);
}
EXPORT_SYMBOL_GPL(rt_mutex_lock);

int __sched rt_mutex_lock_interruptible(struct rt_mutex *lock,
						 int detect_deadlock)
{
	might_sleep();

	return rt_mutex_fastlock(lock, TASK_INTERRUPTIBLE,
				 detect_deadlock, rt_mutex_slowlock);
}
EXPORT_SYMBOL_GPL(rt_mutex_lock_interruptible);

int
rt_mutex_timed_lock(struct rt_mutex *lock, struct hrtimer_sleeper *timeout,
		    int detect_deadlock)
{
	might_sleep();

	return rt_mutex_timed_fastlock(lock, TASK_INTERRUPTIBLE, timeout,
				       detect_deadlock, rt_mutex_slowlock);
}
EXPORT_SYMBOL_GPL(rt_mutex_timed_lock);

int __sched rt_mutex_trylock(struct rt_mutex *lock)
{
	return rt_mutex_fasttrylock(lock, rt_mutex_slowtrylock);
}
EXPORT_SYMBOL_GPL(rt_mutex_trylock);

void __sched rt_mutex_unlock(struct rt_mutex *lock)
{
	rt_mutex_fastunlock(lock, rt_mutex_slowunlock);
}
EXPORT_SYMBOL_GPL(rt_mutex_unlock);

void rt_mutex_destroy(struct rt_mutex *lock)
{
	WARN_ON(rt_mutex_is_locked(lock));
#ifdef CONFIG_DEBUG_RT_MUTEXES
	lock->magic = NULL;
#endif
}

EXPORT_SYMBOL_GPL(rt_mutex_destroy);

void __rt_mutex_init(struct rt_mutex *lock, const char *name)
{
	lock->owner = NULL;
	raw_spin_lock_init(&lock->wait_lock);
	plist_head_init_raw(&lock->wait_list, &lock->wait_lock);

	debug_rt_mutex_init(lock, name);
}
EXPORT_SYMBOL_GPL(__rt_mutex_init);

void rt_mutex_init_proxy_locked(struct rt_mutex *lock,
				struct task_struct *proxy_owner)
{
	__rt_mutex_init(lock, NULL);
	debug_rt_mutex_proxy_lock(lock, proxy_owner);
	rt_mutex_set_owner(lock, proxy_owner, 0);
	rt_mutex_deadlock_account_lock(lock, proxy_owner);
}

void rt_mutex_proxy_unlock(struct rt_mutex *lock,
			   struct task_struct *proxy_owner)
{
	debug_rt_mutex_proxy_unlock(lock);
	rt_mutex_set_owner(lock, NULL, 0);
	rt_mutex_deadlock_account_unlock(proxy_owner);
}

int rt_mutex_start_proxy_lock(struct rt_mutex *lock,
			      struct rt_mutex_waiter *waiter,
			      struct task_struct *task, int detect_deadlock)
{
	int ret;

	raw_spin_lock(&lock->wait_lock);

	mark_rt_mutex_waiters(lock);

	if (!rt_mutex_owner(lock) || try_to_steal_lock(lock, task)) {
		/* We got the lock for task. */
		debug_rt_mutex_lock(lock);
		rt_mutex_set_owner(lock, task, 0);
		raw_spin_unlock(&lock->wait_lock);
		rt_mutex_deadlock_account_lock(lock, task);
		return 1;
	}

	ret = task_blocks_on_rt_mutex(lock, waiter, task, detect_deadlock);

	if (ret && !waiter->task) {
		/*
		 * Reset the return value. We might have
		 * returned with -EDEADLK and the owner
		 * released the lock while we were walking the
		 * pi chain.  Let the waiter sort it out.
		 */
		ret = 0;
	}
	raw_spin_unlock(&lock->wait_lock);

	debug_rt_mutex_print_deadlock(waiter);

	return ret;
}

struct task_struct *rt_mutex_next_owner(struct rt_mutex *lock)
{
	if (!rt_mutex_has_waiters(lock))
		return NULL;

	return rt_mutex_top_waiter(lock)->task;
}

int rt_mutex_finish_proxy_lock(struct rt_mutex *lock,
			       struct hrtimer_sleeper *to,
			       struct rt_mutex_waiter *waiter,
			       int detect_deadlock)
{
	int ret;

	raw_spin_lock(&lock->wait_lock);

	set_current_state(TASK_INTERRUPTIBLE);

	ret = __rt_mutex_slowlock(lock, TASK_INTERRUPTIBLE, to, waiter,
				  detect_deadlock);

	set_current_state(TASK_RUNNING);

	if (unlikely(waiter->task))
		remove_waiter(lock, waiter);

	/*
	 * try_to_take_rt_mutex() sets the waiter bit unconditionally. We might
	 * have to fix that up.
	 */
	fixup_rt_mutex_waiters(lock);

	raw_spin_unlock(&lock->wait_lock);

	/*
	 * Readjust priority, when we did not get the lock. We might have been
	 * the pending owner and boosted. Since we did not take the lock, the
	 * PI boost has to go.
	 */
	if (unlikely(ret))
		rt_mutex_adjust_prio(current);

	return ret;
}
