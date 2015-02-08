
#ifndef __XFS_SUPPORT_SV_H__
#define __XFS_SUPPORT_SV_H__

#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/spinlock.h>


typedef struct sv_s {
	wait_queue_head_t waiters;
} sv_t;

static inline void _sv_wait(sv_t *sv, spinlock_t *lock)
{
	DECLARE_WAITQUEUE(wait, current);

	add_wait_queue_exclusive(&sv->waiters, &wait);
	__set_current_state(TASK_UNINTERRUPTIBLE);
	spin_unlock(lock);

	schedule();

	remove_wait_queue(&sv->waiters, &wait);
}

#define sv_init(sv,flag,name) \
	init_waitqueue_head(&(sv)->waiters)
#define sv_destroy(sv) \
	/*NOTHING*/
#define sv_wait(sv, pri, lock, s) \
	_sv_wait(sv, lock)
#define sv_signal(sv) \
	wake_up(&(sv)->waiters)
#define sv_broadcast(sv) \
	wake_up_all(&(sv)->waiters)

#endif /* __XFS_SUPPORT_SV_H__ */
