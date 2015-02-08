

#ifndef CN_PROC_H
#define CN_PROC_H

#include <linux/types.h>

enum proc_cn_mcast_op {
	PROC_CN_MCAST_LISTEN = 1,
	PROC_CN_MCAST_IGNORE = 2
};


struct proc_event {
	enum what {
		/* Use successive bits so the enums can be used to record
		 * sets of events as well
		 */
		PROC_EVENT_NONE = 0x00000000,
		PROC_EVENT_FORK = 0x00000001,
		PROC_EVENT_EXEC = 0x00000002,
		PROC_EVENT_UID  = 0x00000004,
		PROC_EVENT_GID  = 0x00000040,
		PROC_EVENT_SID  = 0x00000080,
		/* "next" should be 0x00000400 */
		/* "last" is the last process event: exit */
		PROC_EVENT_EXIT = 0x80000000
	} what;
	__u32 cpu;
	__u64 __attribute__((aligned(8))) timestamp_ns;
		/* Number of nano seconds since system boot */
	union { /* must be last field of proc_event struct */
		struct {
			__u32 err;
		} ack;

		struct fork_proc_event {
			__kernel_pid_t parent_pid;
			__kernel_pid_t parent_tgid;
			__kernel_pid_t child_pid;
			__kernel_pid_t child_tgid;
		} fork;

		struct exec_proc_event {
			__kernel_pid_t process_pid;
			__kernel_pid_t process_tgid;
		} exec;

		struct id_proc_event {
			__kernel_pid_t process_pid;
			__kernel_pid_t process_tgid;
			union {
				__u32 ruid; /* task uid */
				__u32 rgid; /* task gid */
			} r;
			union {
				__u32 euid;
				__u32 egid;
			} e;
		} id;

		struct sid_proc_event {
			__kernel_pid_t process_pid;
			__kernel_pid_t process_tgid;
		} sid;

		struct exit_proc_event {
			__kernel_pid_t process_pid;
			__kernel_pid_t process_tgid;
			__u32 exit_code, exit_signal;
		} exit;
	} event_data;
};

#ifdef __KERNEL__
#ifdef CONFIG_PROC_EVENTS
void proc_fork_connector(struct task_struct *task);
void proc_exec_connector(struct task_struct *task);
void proc_id_connector(struct task_struct *task, int which_id);
void proc_sid_connector(struct task_struct *task);
void proc_exit_connector(struct task_struct *task);
#else
static inline void proc_fork_connector(struct task_struct *task)
{}

static inline void proc_exec_connector(struct task_struct *task)
{}

static inline void proc_id_connector(struct task_struct *task,
				     int which_id)
{}

static inline void proc_sid_connector(struct task_struct *task)
{}

static inline void proc_exit_connector(struct task_struct *task)
{}
#endif	/* CONFIG_PROC_EVENTS */
#endif	/* __KERNEL__ */
#endif	/* CN_PROC_H */
