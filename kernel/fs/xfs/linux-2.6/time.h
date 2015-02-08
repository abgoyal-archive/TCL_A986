
#ifndef __XFS_SUPPORT_TIME_H__
#define __XFS_SUPPORT_TIME_H__

#include <linux/sched.h>
#include <linux/time.h>

typedef struct timespec timespec_t;

static inline void delay(long ticks)
{
	schedule_timeout_uninterruptible(ticks);
}

static inline void nanotime(struct timespec *tvp)
{
	*tvp = CURRENT_TIME;
}

#endif /* __XFS_SUPPORT_TIME_H__ */
