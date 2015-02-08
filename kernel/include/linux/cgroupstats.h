

#ifndef _LINUX_CGROUPSTATS_H
#define _LINUX_CGROUPSTATS_H

#include <linux/types.h>
#include <linux/taskstats.h>

struct cgroupstats {
	__u64	nr_sleeping;		/* Number of tasks sleeping */
	__u64	nr_running;		/* Number of tasks running */
	__u64	nr_stopped;		/* Number of tasks in stopped state */
	__u64	nr_uninterruptible;	/* Number of tasks in uninterruptible */
					/* state */
	__u64	nr_io_wait;		/* Number of tasks waiting on IO */
};


enum {
	CGROUPSTATS_CMD_UNSPEC = __TASKSTATS_CMD_MAX,	/* Reserved */
	CGROUPSTATS_CMD_GET,		/* user->kernel request/get-response */
	CGROUPSTATS_CMD_NEW,		/* kernel->user event */
	__CGROUPSTATS_CMD_MAX,
};

#define CGROUPSTATS_CMD_MAX (__CGROUPSTATS_CMD_MAX - 1)

enum {
	CGROUPSTATS_TYPE_UNSPEC = 0,	/* Reserved */
	CGROUPSTATS_TYPE_CGROUP_STATS,	/* contains name + stats */
	__CGROUPSTATS_TYPE_MAX,
};

#define CGROUPSTATS_TYPE_MAX (__CGROUPSTATS_TYPE_MAX - 1)

enum {
	CGROUPSTATS_CMD_ATTR_UNSPEC = 0,
	CGROUPSTATS_CMD_ATTR_FD,
	__CGROUPSTATS_CMD_ATTR_MAX,
};

#define CGROUPSTATS_CMD_ATTR_MAX (__CGROUPSTATS_CMD_ATTR_MAX - 1)

#endif /* _LINUX_CGROUPSTATS_H */
