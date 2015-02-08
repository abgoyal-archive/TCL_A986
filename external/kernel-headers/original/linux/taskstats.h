
#ifndef _LINUX_TASKSTATS_H
#define _LINUX_TASKSTATS_H


#define TASKSTATS_VERSION	1

struct taskstats {

	/* Version 1 */
	__u16	version;
	__u16	padding[3];	/* Userspace should not interpret the padding
				 * field which can be replaced by useful
				 * fields if struct taskstats is extended.
				 */

	/* Delay accounting fields start
	 *
	 * All values, until comment "Delay accounting fields end" are
	 * available only if delay accounting is enabled, even though the last
	 * few fields are not delays
	 *
	 * xxx_count is the number of delay values recorded
	 * xxx_delay_total is the corresponding cumulative delay in nanoseconds
	 *
	 * xxx_delay_total wraps around to zero on overflow
	 * xxx_count incremented regardless of overflow
	 */

	/* Delay waiting for cpu, while runnable
	 * count, delay_total NOT updated atomically
	 */
	__u64	cpu_count;
	__u64	cpu_delay_total;

	/* Following four fields atomically updated using task->delays->lock */

	/* Delay waiting for synchronous block I/O to complete
	 * does not account for delays in I/O submission
	 */
	__u64	blkio_count;
	__u64	blkio_delay_total;

	/* Delay waiting for page fault I/O (swap in only) */
	__u64	swapin_count;
	__u64	swapin_delay_total;

	/* cpu "wall-clock" running time
	 * On some architectures, value will adjust for cpu time stolen
	 * from the kernel in involuntary waits due to virtualization.
	 * Value is cumulative, in nanoseconds, without a corresponding count
	 * and wraps around to zero silently on overflow
	 */
	__u64	cpu_run_real_total;

	/* cpu "virtual" running time
	 * Uses time intervals seen by the kernel i.e. no adjustment
	 * for kernel's involuntary waits due to virtualization.
	 * Value is cumulative, in nanoseconds, without a corresponding count
	 * and wraps around to zero silently on overflow
	 */
	__u64	cpu_run_virtual_total;
	/* Delay accounting fields end */
	/* version 1 ends here */
};



enum {
	TASKSTATS_CMD_UNSPEC = 0,	/* Reserved */
	TASKSTATS_CMD_GET,		/* user->kernel request/get-response */
	TASKSTATS_CMD_NEW,		/* kernel->user event */
	__TASKSTATS_CMD_MAX,
};

#define TASKSTATS_CMD_MAX (__TASKSTATS_CMD_MAX - 1)

enum {
	TASKSTATS_TYPE_UNSPEC = 0,	/* Reserved */
	TASKSTATS_TYPE_PID,		/* Process id */
	TASKSTATS_TYPE_TGID,		/* Thread group id */
	TASKSTATS_TYPE_STATS,		/* taskstats structure */
	TASKSTATS_TYPE_AGGR_PID,	/* contains pid + stats */
	TASKSTATS_TYPE_AGGR_TGID,	/* contains tgid + stats */
	__TASKSTATS_TYPE_MAX,
};

#define TASKSTATS_TYPE_MAX (__TASKSTATS_TYPE_MAX - 1)

enum {
	TASKSTATS_CMD_ATTR_UNSPEC = 0,
	TASKSTATS_CMD_ATTR_PID,
	TASKSTATS_CMD_ATTR_TGID,
	TASKSTATS_CMD_ATTR_REGISTER_CPUMASK,
	TASKSTATS_CMD_ATTR_DEREGISTER_CPUMASK,
	__TASKSTATS_CMD_ATTR_MAX,
};

#define TASKSTATS_CMD_ATTR_MAX (__TASKSTATS_CMD_ATTR_MAX - 1)

/* NETLINK_GENERIC related info */

#define TASKSTATS_GENL_NAME	"TASKSTATS"
#define TASKSTATS_GENL_VERSION	0x1

#endif /* _LINUX_TASKSTATS_H */
