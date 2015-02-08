
#ifndef _H_JFS_LOCK
#define _H_JFS_LOCK

#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/sched.h>


#define __SLEEP_COND(wq, cond, lock_cmd, unlock_cmd)	\
do {							\
	DECLARE_WAITQUEUE(__wait, current);		\
							\
	add_wait_queue(&wq, &__wait);			\
	for (;;) {					\
		set_current_state(TASK_UNINTERRUPTIBLE);\
		if (cond)				\
			break;				\
		unlock_cmd;				\
		io_schedule();				\
		lock_cmd;				\
	}						\
	__set_current_state(TASK_RUNNING);			\
	remove_wait_queue(&wq, &__wait);		\
} while (0)

#endif				/* _H_JFS_LOCK */
