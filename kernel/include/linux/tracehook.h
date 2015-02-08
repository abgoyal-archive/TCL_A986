

#ifndef _LINUX_TRACEHOOK_H
#define _LINUX_TRACEHOOK_H	1

#include <linux/sched.h>
#include <linux/ptrace.h>
#include <linux/security.h>
struct linux_binprm;

static inline int tracehook_expect_breakpoints(struct task_struct *task)
{
	return (task_ptrace(task) & PT_PTRACED) != 0;
}

static inline void ptrace_report_syscall(struct pt_regs *regs)
{
	int ptrace = task_ptrace(current);

	if (!(ptrace & PT_PTRACED))
		return;

	ptrace_notify(SIGTRAP | ((ptrace & PT_TRACESYSGOOD) ? 0x80 : 0));

	/*
	 * this isn't the same as continuing with a signal, but it will do
	 * for normal use.  strace only continues with a signal if the
	 * stopping signal is not SIGTRAP.  -brl
	 */
	if (current->exit_code) {
		send_sig(current->exit_code, current, 1);
		current->exit_code = 0;
	}
}

static inline __must_check int tracehook_report_syscall_entry(
	struct pt_regs *regs)
{
	ptrace_report_syscall(regs);
	return 0;
}

static inline void tracehook_report_syscall_exit(struct pt_regs *regs, int step)
{
	if (step) {
		siginfo_t info;
		user_single_step_siginfo(current, regs, &info);
		force_sig_info(SIGTRAP, &info, current);
		return;
	}

	ptrace_report_syscall(regs);
}

static inline int tracehook_unsafe_exec(struct task_struct *task)
{
	int unsafe = 0;
	int ptrace = task_ptrace(task);
	if (ptrace & PT_PTRACED) {
		if (ptrace & PT_PTRACE_CAP)
			unsafe |= LSM_UNSAFE_PTRACE_CAP;
		else
			unsafe |= LSM_UNSAFE_PTRACE;
	}
	return unsafe;
}

static inline struct task_struct *tracehook_tracer_task(struct task_struct *tsk)
{
	if (task_ptrace(tsk) & PT_PTRACED)
		return rcu_dereference(tsk->parent);
	return NULL;
}

static inline void tracehook_report_exec(struct linux_binfmt *fmt,
					 struct linux_binprm *bprm,
					 struct pt_regs *regs)
{
	if (!ptrace_event(PT_TRACE_EXEC, PTRACE_EVENT_EXEC, 0) &&
	    unlikely(task_ptrace(current) & PT_PTRACED))
		send_sig(SIGTRAP, current, 0);
}

static inline void tracehook_report_exit(long *exit_code)
{
	ptrace_event(PT_TRACE_EXIT, PTRACE_EVENT_EXIT, *exit_code);
}

static inline int tracehook_prepare_clone(unsigned clone_flags)
{
	if (clone_flags & CLONE_UNTRACED)
		return 0;

	if (clone_flags & CLONE_VFORK) {
		if (current->ptrace & PT_TRACE_VFORK)
			return PTRACE_EVENT_VFORK;
	} else if ((clone_flags & CSIGNAL) != SIGCHLD) {
		if (current->ptrace & PT_TRACE_CLONE)
			return PTRACE_EVENT_CLONE;
	} else if (current->ptrace & PT_TRACE_FORK)
		return PTRACE_EVENT_FORK;

	return 0;
}

static inline void tracehook_finish_clone(struct task_struct *child,
					  unsigned long clone_flags, int trace)
{
	ptrace_init_task(child, (clone_flags & CLONE_PTRACE) || trace);
}

static inline void tracehook_report_clone(struct pt_regs *regs,
					  unsigned long clone_flags,
					  pid_t pid, struct task_struct *child)
{
	if (unlikely(task_ptrace(child))) {
		/*
		 * It doesn't matter who attached/attaching to this
		 * task, the pending SIGSTOP is right in any case.
		 */
		sigaddset(&child->pending.signal, SIGSTOP);
		set_tsk_thread_flag(child, TIF_SIGPENDING);
	}
}

static inline void tracehook_report_clone_complete(int trace,
						   struct pt_regs *regs,
						   unsigned long clone_flags,
						   pid_t pid,
						   struct task_struct *child)
{
	if (unlikely(trace))
		ptrace_event(0, trace, pid);
}

static inline void tracehook_report_vfork_done(struct task_struct *child,
					       pid_t pid)
{
	ptrace_event(PT_TRACE_VFORK_DONE, PTRACE_EVENT_VFORK_DONE, pid);
}

static inline void tracehook_prepare_release_task(struct task_struct *task)
{
}

static inline void tracehook_finish_release_task(struct task_struct *task)
{
	ptrace_release_task(task);
}

static inline void tracehook_signal_handler(int sig, siginfo_t *info,
					    const struct k_sigaction *ka,
					    struct pt_regs *regs, int stepping)
{
	if (stepping)
		ptrace_notify(SIGTRAP);
}

static inline int tracehook_consider_ignored_signal(struct task_struct *task,
						    int sig)
{
	return (task_ptrace(task) & PT_PTRACED) != 0;
}

static inline int tracehook_consider_fatal_signal(struct task_struct *task,
						  int sig)
{
	return (task_ptrace(task) & PT_PTRACED) != 0;
}

static inline int tracehook_force_sigpending(void)
{
	return 0;
}

static inline int tracehook_get_signal(struct task_struct *task,
				       struct pt_regs *regs,
				       siginfo_t *info,
				       struct k_sigaction *return_ka)
{
	return 0;
}

static inline int tracehook_notify_jctl(int notify, int why)
{
	return notify ?: (current->ptrace & PT_PTRACED) ? why : 0;
}

static inline void tracehook_finish_jctl(void)
{
}

#define DEATH_REAP			-1
#define DEATH_DELAYED_GROUP_LEADER	-2

static inline int tracehook_notify_death(struct task_struct *task,
					 void **death_cookie, int group_dead)
{
	if (task_detached(task))
		return task->ptrace ? SIGCHLD : DEATH_REAP;

	/*
	 * If something other than our normal parent is ptracing us, then
	 * send it a SIGCHLD instead of honoring exit_signal.  exit_signal
	 * only has special meaning to our real parent.
	 */
	if (thread_group_empty(task) && !ptrace_reparented(task))
		return task->exit_signal;

	return task->ptrace ? SIGCHLD : DEATH_DELAYED_GROUP_LEADER;
}

static inline void tracehook_report_death(struct task_struct *task,
					  int signal, void *death_cookie,
					  int group_dead)
{
}

#ifdef TIF_NOTIFY_RESUME
static inline void set_notify_resume(struct task_struct *task)
{
	if (!test_and_set_tsk_thread_flag(task, TIF_NOTIFY_RESUME))
		kick_process(task);
}

static inline void tracehook_notify_resume(struct pt_regs *regs)
{
}
#endif	/* TIF_NOTIFY_RESUME */

#endif	/* <linux/tracehook.h> */
