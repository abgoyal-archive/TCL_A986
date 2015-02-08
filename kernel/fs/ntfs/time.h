

#ifndef _LINUX_NTFS_TIME_H
#define _LINUX_NTFS_TIME_H

#include <linux/time.h>		/* For current_kernel_time(). */
#include <asm/div64.h>		/* For do_div(). */

#include "endian.h"

#define NTFS_TIME_OFFSET ((s64)(369 * 365 + 89) * 24 * 3600 * 10000000)

static inline sle64 utc2ntfs(const struct timespec ts)
{
	/*
	 * Convert the seconds to 100ns intervals, add the nano-seconds
	 * converted to 100ns intervals, and then add the NTFS time offset.
	 */
	return cpu_to_sle64((s64)ts.tv_sec * 10000000 + ts.tv_nsec / 100 +
			NTFS_TIME_OFFSET);
}

static inline sle64 get_current_ntfs_time(void)
{
	return utc2ntfs(current_kernel_time());
}

static inline struct timespec ntfs2utc(const sle64 time)
{
	struct timespec ts;

	/* Subtract the NTFS time offset. */
	u64 t = (u64)(sle64_to_cpu(time) - NTFS_TIME_OFFSET);
	/*
	 * Convert the time to 1-second intervals and the remainder to
	 * 1-nano-second intervals.
	 */
	ts.tv_nsec = do_div(t, 10000000) * 100;
	ts.tv_sec = t;
	return ts;
}

#endif /* _LINUX_NTFS_TIME_H */
