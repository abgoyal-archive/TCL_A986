#ifndef _ASM_GENERIC_MUTEX_XCHG_H
#define _ASM_GENERIC_MUTEX_XCHG_H

static inline void
__mutex_fastpath_lock(atomic_t *count, fastcall void (*fail_fn)(atomic_t *))
{
	if (unlikely(atomic_xchg(count, 0) != 1))
		fail_fn(count);
	else
		smp_mb();
}

static inline int
__mutex_fastpath_lock_retval(atomic_t *count, fastcall int (*fail_fn)(atomic_t *))
{
	if (unlikely(atomic_xchg(count, 0) != 1))
		return fail_fn(count);
	else {
		smp_mb();
		return 0;
	}
}

static inline void
__mutex_fastpath_unlock(atomic_t *count, fastcall void (*fail_fn)(atomic_t *))
{
	smp_mb();
	if (unlikely(atomic_xchg(count, 1) != 0))
		fail_fn(count);
}

#define __mutex_slowpath_needs_to_unlock()		0

static inline int
__mutex_fastpath_trylock(atomic_t *count, int (*fail_fn)(atomic_t *))
{
	int prev = atomic_xchg(count, 0);

	if (unlikely(prev < 0)) {
		/*
		 * The lock was marked contended so we must restore that
		 * state. If while doing so we get back a prev value of 1
		 * then we just own it.
		 *
		 * [ In the rare case of the mutex going to 1, to 0, to -1
		 *   and then back to 0 in this few-instructions window,
		 *   this has the potential to trigger the slowpath for the
		 *   owner's unlock path needlessly, but that's not a problem
		 *   in practice. ]
		 */
		prev = atomic_xchg(count, prev);
		if (prev < 0)
			prev = 0;
	}
	smp_mb();

	return prev;
}

#endif
