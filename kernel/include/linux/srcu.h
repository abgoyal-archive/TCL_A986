

#ifndef _LINUX_SRCU_H
#define _LINUX_SRCU_H

#include <linux/mutex.h>

struct srcu_struct_array {
	int c[2];
};

struct srcu_struct {
	int completed;
	struct srcu_struct_array __percpu *per_cpu_ref;
	struct mutex mutex;
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	struct lockdep_map dep_map;
#endif /* #ifdef CONFIG_DEBUG_LOCK_ALLOC */
};

#ifndef CONFIG_PREEMPT
#define srcu_barrier() barrier()
#else /* #ifndef CONFIG_PREEMPT */
#define srcu_barrier()
#endif /* #else #ifndef CONFIG_PREEMPT */

#ifdef CONFIG_DEBUG_LOCK_ALLOC

int __init_srcu_struct(struct srcu_struct *sp, const char *name,
		       struct lock_class_key *key);

#define init_srcu_struct(sp) \
({ \
	static struct lock_class_key __srcu_key; \
	\
	__init_srcu_struct((sp), #sp, &__srcu_key); \
})

# define srcu_read_acquire(sp) \
		lock_acquire(&(sp)->dep_map, 0, 0, 2, 1, NULL, _THIS_IP_)
# define srcu_read_release(sp) \
		lock_release(&(sp)->dep_map, 1, _THIS_IP_)

#else /* #ifdef CONFIG_DEBUG_LOCK_ALLOC */

int init_srcu_struct(struct srcu_struct *sp);

# define srcu_read_acquire(sp)  do { } while (0)
# define srcu_read_release(sp)  do { } while (0)

#endif /* #else #ifdef CONFIG_DEBUG_LOCK_ALLOC */

void cleanup_srcu_struct(struct srcu_struct *sp);
int __srcu_read_lock(struct srcu_struct *sp) __acquires(sp);
void __srcu_read_unlock(struct srcu_struct *sp, int idx) __releases(sp);
void synchronize_srcu(struct srcu_struct *sp);
void synchronize_srcu_expedited(struct srcu_struct *sp);
long srcu_batches_completed(struct srcu_struct *sp);

#ifdef CONFIG_DEBUG_LOCK_ALLOC

static inline int srcu_read_lock_held(struct srcu_struct *sp)
{
	if (debug_locks)
		return lock_is_held(&sp->dep_map);
	return 1;
}

#else /* #ifdef CONFIG_DEBUG_LOCK_ALLOC */

static inline int srcu_read_lock_held(struct srcu_struct *sp)
{
	return 1;
}

#endif /* #else #ifdef CONFIG_DEBUG_LOCK_ALLOC */

#define srcu_dereference(p, sp) \
		rcu_dereference_check(p, srcu_read_lock_held(sp))

static inline int srcu_read_lock(struct srcu_struct *sp) __acquires(sp)
{
	int retval = __srcu_read_lock(sp);

	srcu_read_acquire(sp);
	return retval;
}

static inline void srcu_read_unlock(struct srcu_struct *sp, int idx)
	__releases(sp)
{
	srcu_read_release(sp);
	__srcu_read_unlock(sp, idx);
}

#endif
