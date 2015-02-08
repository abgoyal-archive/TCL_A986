

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/percpu.h>
#include <linux/preempt.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/srcu.h>

static int init_srcu_struct_fields(struct srcu_struct *sp)
{
	sp->completed = 0;
	mutex_init(&sp->mutex);
	sp->per_cpu_ref = alloc_percpu(struct srcu_struct_array);
	return sp->per_cpu_ref ? 0 : -ENOMEM;
}

#ifdef CONFIG_DEBUG_LOCK_ALLOC

int __init_srcu_struct(struct srcu_struct *sp, const char *name,
		       struct lock_class_key *key)
{
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	/* Don't re-initialize a lock while it is held. */
	debug_check_no_locks_freed((void *)sp, sizeof(*sp));
	lockdep_init_map(&sp->dep_map, name, key, 0);
#endif /* #ifdef CONFIG_DEBUG_LOCK_ALLOC */
	return init_srcu_struct_fields(sp);
}
EXPORT_SYMBOL_GPL(__init_srcu_struct);

#else /* #ifdef CONFIG_DEBUG_LOCK_ALLOC */

int init_srcu_struct(struct srcu_struct *sp)
{
	return init_srcu_struct_fields(sp);
}
EXPORT_SYMBOL_GPL(init_srcu_struct);

#endif /* #else #ifdef CONFIG_DEBUG_LOCK_ALLOC */


static int srcu_readers_active_idx(struct srcu_struct *sp, int idx)
{
	int cpu;
	int sum;

	sum = 0;
	for_each_possible_cpu(cpu)
		sum += per_cpu_ptr(sp->per_cpu_ref, cpu)->c[idx];
	return sum;
}

static int srcu_readers_active(struct srcu_struct *sp)
{
	return srcu_readers_active_idx(sp, 0) + srcu_readers_active_idx(sp, 1);
}

void cleanup_srcu_struct(struct srcu_struct *sp)
{
	int sum;

	sum = srcu_readers_active(sp);
	WARN_ON(sum);  /* Leakage unless caller handles error. */
	if (sum != 0)
		return;
	free_percpu(sp->per_cpu_ref);
	sp->per_cpu_ref = NULL;
}
EXPORT_SYMBOL_GPL(cleanup_srcu_struct);

int __srcu_read_lock(struct srcu_struct *sp)
{
	int idx;

	preempt_disable();
	idx = sp->completed & 0x1;
	barrier();  /* ensure compiler looks -once- at sp->completed. */
	per_cpu_ptr(sp->per_cpu_ref, smp_processor_id())->c[idx]++;
	srcu_barrier();  /* ensure compiler won't misorder critical section. */
	preempt_enable();
	return idx;
}
EXPORT_SYMBOL_GPL(__srcu_read_lock);

void __srcu_read_unlock(struct srcu_struct *sp, int idx)
{
	preempt_disable();
	srcu_barrier();  /* ensure compiler won't misorder critical section. */
	per_cpu_ptr(sp->per_cpu_ref, smp_processor_id())->c[idx]--;
	preempt_enable();
}
EXPORT_SYMBOL_GPL(__srcu_read_unlock);

static void __synchronize_srcu(struct srcu_struct *sp, void (*sync_func)(void))
{
	int idx;

	idx = sp->completed;
	mutex_lock(&sp->mutex);

	/*
	 * Check to see if someone else did the work for us while we were
	 * waiting to acquire the lock.  We need -two- advances of
	 * the counter, not just one.  If there was but one, we might have
	 * shown up -after- our helper's first synchronize_sched(), thus
	 * having failed to prevent CPU-reordering races with concurrent
	 * srcu_read_unlock()s on other CPUs (see comment below).  So we
	 * either (1) wait for two or (2) supply the second ourselves.
	 */

	if ((sp->completed - idx) >= 2) {
		mutex_unlock(&sp->mutex);
		return;
	}

	sync_func();  /* Force memory barrier on all CPUs. */

	/*
	 * The preceding synchronize_sched() ensures that any CPU that
	 * sees the new value of sp->completed will also see any preceding
	 * changes to data structures made by this CPU.  This prevents
	 * some other CPU from reordering the accesses in its SRCU
	 * read-side critical section to precede the corresponding
	 * srcu_read_lock() -- ensuring that such references will in
	 * fact be protected.
	 *
	 * So it is now safe to do the flip.
	 */

	idx = sp->completed & 0x1;
	sp->completed++;

	sync_func();  /* Force memory barrier on all CPUs. */

	/*
	 * At this point, because of the preceding synchronize_sched(),
	 * all srcu_read_lock() calls using the old counters have completed.
	 * Their corresponding critical sections might well be still
	 * executing, but the srcu_read_lock() primitives themselves
	 * will have finished executing.
	 */

	while (srcu_readers_active_idx(sp, idx))
		schedule_timeout_interruptible(1);

	sync_func();  /* Force memory barrier on all CPUs. */

	/*
	 * The preceding synchronize_sched() forces all srcu_read_unlock()
	 * primitives that were executing concurrently with the preceding
	 * for_each_possible_cpu() loop to have completed by this point.
	 * More importantly, it also forces the corresponding SRCU read-side
	 * critical sections to have also completed, and the corresponding
	 * references to SRCU-protected data items to be dropped.
	 *
	 * Note:
	 *
	 *	Despite what you might think at first glance, the
	 *	preceding synchronize_sched() -must- be within the
	 *	critical section ended by the following mutex_unlock().
	 *	Otherwise, a task taking the early exit can race
	 *	with a srcu_read_unlock(), which might have executed
	 *	just before the preceding srcu_readers_active() check,
	 *	and whose CPU might have reordered the srcu_read_unlock()
	 *	with the preceding critical section.  In this case, there
	 *	is nothing preventing the synchronize_sched() task that is
	 *	taking the early exit from freeing a data structure that
	 *	is still being referenced (out of order) by the task
	 *	doing the srcu_read_unlock().
	 *
	 *	Alternatively, the comparison with "2" on the early exit
	 *	could be changed to "3", but this increases synchronize_srcu()
	 *	latency for bulk loads.  So the current code is preferred.
	 */

	mutex_unlock(&sp->mutex);
}

void synchronize_srcu(struct srcu_struct *sp)
{
	__synchronize_srcu(sp, synchronize_sched);
}
EXPORT_SYMBOL_GPL(synchronize_srcu);

void synchronize_srcu_expedited(struct srcu_struct *sp)
{
	__synchronize_srcu(sp, synchronize_sched_expedited);
}
EXPORT_SYMBOL_GPL(synchronize_srcu_expedited);


long srcu_batches_completed(struct srcu_struct *sp)
{
	return sp->completed;
}
EXPORT_SYMBOL_GPL(srcu_batches_completed);
