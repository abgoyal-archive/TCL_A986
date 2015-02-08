
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/debug_locks.h>

#ifdef CONFIG_DEBUG_MUTEXES
# include "mutex-debug.h"
# include <asm-generic/mutex-null.h>
#else
# include "mutex.h"
# include <asm/mutex.h>
#endif

void
__mutex_init(struct mutex *lock, const char *name, struct lock_class_key *key)
{
	atomic_set(&lock->count, 1);
	spin_lock_init(&lock->wait_lock);
	INIT_LIST_HEAD(&lock->wait_list);
	mutex_clear_owner(lock);

	debug_mutex_init(lock, name, key);
}

EXPORT_SYMBOL(__mutex_init);

#ifndef CONFIG_DEBUG_LOCK_ALLOC
static __used noinline void __sched
__mutex_lock_slowpath(atomic_t *lock_count);

void __sched mutex_lock(struct mutex *lock)
{
	might_sleep();
	/*
	 * The locking fastpath is the 1->0 transition from
	 * 'unlocked' into 'locked' state.
	 */
	__mutex_fastpath_lock(&lock->count, __mutex_lock_slowpath);
	mutex_set_owner(lock);
}

EXPORT_SYMBOL(mutex_lock);
#endif

static __used noinline void __sched __mutex_unlock_slowpath(atomic_t *lock_count);

void __sched mutex_unlock(struct mutex *lock)
{
	/*
	 * The unlocking fastpath is the 0->1 transition from 'locked'
	 * into 'unlocked' state:
	 */
#ifndef CONFIG_DEBUG_MUTEXES
	/*
	 * When debugging is enabled we must not clear the owner before time,
	 * the slow path will always be taken, and that clears the owner field
	 * after verifying that it was indeed current.
	 */
	mutex_clear_owner(lock);
#endif
	__mutex_fastpath_unlock(&lock->count, __mutex_unlock_slowpath);
}

EXPORT_SYMBOL(mutex_unlock);

static inline int __sched
__mutex_lock_common(struct mutex *lock, long state, unsigned int subclass,
	       	unsigned long ip)
{
	struct task_struct *task = current;
	struct mutex_waiter waiter;
	unsigned long flags;

	preempt_disable();
	mutex_acquire(&lock->dep_map, subclass, 0, ip);

#ifdef CONFIG_MUTEX_SPIN_ON_OWNER
	/*
	 * Optimistic spinning.
	 *
	 * We try to spin for acquisition when we find that there are no
	 * pending waiters and the lock owner is currently running on a
	 * (different) CPU.
	 *
	 * The rationale is that if the lock owner is running, it is likely to
	 * release the lock soon.
	 *
	 * Since this needs the lock owner, and this mutex implementation
	 * doesn't track the owner atomically in the lock field, we need to
	 * track it non-atomically.
	 *
	 * We can't do this for DEBUG_MUTEXES because that relies on wait_lock
	 * to serialize everything.
	 */

	for (;;) {
		struct thread_info *owner;

		/*
		 * If we own the BKL, then don't spin. The owner of
		 * the mutex might be waiting on us to release the BKL.
		 */
		if (unlikely(current->lock_depth >= 0))
			break;

		/*
		 * If there's an owner, wait for it to either
		 * release the lock or go to sleep.
		 */
		owner = ACCESS_ONCE(lock->owner);
		if (owner && !mutex_spin_on_owner(lock, owner))
			break;

		if (atomic_cmpxchg(&lock->count, 1, 0) == 1) {
			lock_acquired(&lock->dep_map, ip);
			mutex_set_owner(lock);
			preempt_enable();
			return 0;
		}

		/*
		 * When there's no owner, we might have preempted between the
		 * owner acquiring the lock and setting the owner field. If
		 * we're an RT task that will live-lock because we won't let
		 * the owner complete.
		 */
		if (!owner && (need_resched() || rt_task(task)))
			break;

		/*
		 * The cpu_relax() call is a compiler barrier which forces
		 * everything in this loop to be re-loaded. We don't need
		 * memory barriers as we'll eventually observe the right
		 * values at the cost of a few extra spins.
		 */
		cpu_relax();
	}
#endif
	spin_lock_mutex(&lock->wait_lock, flags);

	debug_mutex_lock_common(lock, &waiter);
	debug_mutex_add_waiter(lock, &waiter, task_thread_info(task));

	/* add waiting tasks to the end of the waitqueue (FIFO): */
	list_add_tail(&waiter.list, &lock->wait_list);
	waiter.task = task;

	if (atomic_xchg(&lock->count, -1) == 1)
		goto done;

	lock_contended(&lock->dep_map, ip);

	for (;;) {
		/*
		 * Lets try to take the lock again - this is needed even if
		 * we get here for the first time (shortly after failing to
		 * acquire the lock), to make sure that we get a wakeup once
		 * it's unlocked. Later on, if we sleep, this is the
		 * operation that gives us the lock. We xchg it to -1, so
		 * that when we release the lock, we properly wake up the
		 * other waiters:
		 */
		if (atomic_xchg(&lock->count, -1) == 1)
			break;

		/*
		 * got a signal? (This code gets eliminated in the
		 * TASK_UNINTERRUPTIBLE case.)
		 */
		if (unlikely(signal_pending_state(state, task))) {
			mutex_remove_waiter(lock, &waiter,
					    task_thread_info(task));
			mutex_release(&lock->dep_map, 1, ip);
			spin_unlock_mutex(&lock->wait_lock, flags);

			debug_mutex_free_waiter(&waiter);
			preempt_enable();
			return -EINTR;
		}
		__set_task_state(task, state);

		/* didnt get the lock, go to sleep: */
		spin_unlock_mutex(&lock->wait_lock, flags);
		preempt_enable_no_resched();
		schedule();
		preempt_disable();
		spin_lock_mutex(&lock->wait_lock, flags);
	}

done:
	lock_acquired(&lock->dep_map, ip);
	/* got the lock - rejoice! */
	mutex_remove_waiter(lock, &waiter, current_thread_info());
	mutex_set_owner(lock);

	/* set it to 0 if there are no waiters left: */
	if (likely(list_empty(&lock->wait_list)))
		atomic_set(&lock->count, 0);

	spin_unlock_mutex(&lock->wait_lock, flags);

	debug_mutex_free_waiter(&waiter);
	preempt_enable();

	return 0;
}

#ifdef CONFIG_DEBUG_LOCK_ALLOC
void __sched
mutex_lock_nested(struct mutex *lock, unsigned int subclass)
{
	might_sleep();
	__mutex_lock_common(lock, TASK_UNINTERRUPTIBLE, subclass, _RET_IP_);
}

EXPORT_SYMBOL_GPL(mutex_lock_nested);

int __sched
mutex_lock_killable_nested(struct mutex *lock, unsigned int subclass)
{
	might_sleep();
	return __mutex_lock_common(lock, TASK_KILLABLE, subclass, _RET_IP_);
}
EXPORT_SYMBOL_GPL(mutex_lock_killable_nested);

int __sched
mutex_lock_interruptible_nested(struct mutex *lock, unsigned int subclass)
{
	might_sleep();
	return __mutex_lock_common(lock, TASK_INTERRUPTIBLE,
				   subclass, _RET_IP_);
}

EXPORT_SYMBOL_GPL(mutex_lock_interruptible_nested);
#endif

static inline void
__mutex_unlock_common_slowpath(atomic_t *lock_count, int nested)
{
	struct mutex *lock = container_of(lock_count, struct mutex, count);
	unsigned long flags;

	spin_lock_mutex(&lock->wait_lock, flags);
	mutex_release(&lock->dep_map, nested, _RET_IP_);
	debug_mutex_unlock(lock);

	/*
	 * some architectures leave the lock unlocked in the fastpath failure
	 * case, others need to leave it locked. In the later case we have to
	 * unlock it here
	 */
	if (__mutex_slowpath_needs_to_unlock())
		atomic_set(&lock->count, 1);

	if (!list_empty(&lock->wait_list)) {
		/* get the first entry from the wait-list: */
		struct mutex_waiter *waiter =
				list_entry(lock->wait_list.next,
					   struct mutex_waiter, list);

		debug_mutex_wake_waiter(lock, waiter);

		wake_up_process(waiter->task);
	}

	spin_unlock_mutex(&lock->wait_lock, flags);
}

static __used noinline void
__mutex_unlock_slowpath(atomic_t *lock_count)
{
	__mutex_unlock_common_slowpath(lock_count, 1);
}

#ifndef CONFIG_DEBUG_LOCK_ALLOC
static noinline int __sched
__mutex_lock_killable_slowpath(atomic_t *lock_count);

static noinline int __sched
__mutex_lock_interruptible_slowpath(atomic_t *lock_count);

int __sched mutex_lock_interruptible(struct mutex *lock)
{
	int ret;

	might_sleep();
	ret =  __mutex_fastpath_lock_retval
			(&lock->count, __mutex_lock_interruptible_slowpath);
	if (!ret)
		mutex_set_owner(lock);

	return ret;
}

EXPORT_SYMBOL(mutex_lock_interruptible);

int __sched mutex_lock_killable(struct mutex *lock)
{
	int ret;

	might_sleep();
	ret = __mutex_fastpath_lock_retval
			(&lock->count, __mutex_lock_killable_slowpath);
	if (!ret)
		mutex_set_owner(lock);

	return ret;
}
EXPORT_SYMBOL(mutex_lock_killable);

static __used noinline void __sched
__mutex_lock_slowpath(atomic_t *lock_count)
{
	struct mutex *lock = container_of(lock_count, struct mutex, count);

	__mutex_lock_common(lock, TASK_UNINTERRUPTIBLE, 0, _RET_IP_);
}

static noinline int __sched
__mutex_lock_killable_slowpath(atomic_t *lock_count)
{
	struct mutex *lock = container_of(lock_count, struct mutex, count);

	return __mutex_lock_common(lock, TASK_KILLABLE, 0, _RET_IP_);
}

static noinline int __sched
__mutex_lock_interruptible_slowpath(atomic_t *lock_count)
{
	struct mutex *lock = container_of(lock_count, struct mutex, count);

	return __mutex_lock_common(lock, TASK_INTERRUPTIBLE, 0, _RET_IP_);
}
#endif

static inline int __mutex_trylock_slowpath(atomic_t *lock_count)
{
	struct mutex *lock = container_of(lock_count, struct mutex, count);
	unsigned long flags;
	int prev;

	spin_lock_mutex(&lock->wait_lock, flags);

	prev = atomic_xchg(&lock->count, -1);
	if (likely(prev == 1)) {
		mutex_set_owner(lock);
		mutex_acquire(&lock->dep_map, 0, 1, _RET_IP_);
	}

	/* Set it back to 0 if there are no waiters: */
	if (likely(list_empty(&lock->wait_list)))
		atomic_set(&lock->count, 0);

	spin_unlock_mutex(&lock->wait_lock, flags);

	return prev == 1;
}

int __sched mutex_trylock(struct mutex *lock)
{
	int ret;

	ret = __mutex_fastpath_trylock(&lock->count, __mutex_trylock_slowpath);
	if (ret)
		mutex_set_owner(lock);

	return ret;
}
EXPORT_SYMBOL(mutex_trylock);

int atomic_dec_and_mutex_lock(atomic_t *cnt, struct mutex *lock)
{
	/* dec if we can't possibly hit 0 */
	if (atomic_add_unless(cnt, -1, 1))
		return 0;
	/* we might hit 0, so take the lock */
	mutex_lock(lock);
	if (!atomic_dec_and_test(cnt)) {
		/* when we actually did the dec, we didn't hit 0 */
		mutex_unlock(lock);
		return 0;
	}
	/* we hit 0, and we hold the lock */
	return 1;
}
EXPORT_SYMBOL(atomic_dec_and_mutex_lock);
