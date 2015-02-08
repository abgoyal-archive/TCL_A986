
#include <linux/ptrace.h>

extern void ptrace_cancel_bpt(struct task_struct *);
extern void ptrace_set_bpt(struct task_struct *);
extern void ptrace_break(struct task_struct *, struct pt_regs *);

static inline void single_step_trap(struct task_struct *task)
{
	if (task->ptrace & PT_SINGLESTEP) {
		ptrace_cancel_bpt(task);
		send_sig(SIGTRAP, task, 1);
	}
}

static inline void single_step_clear(struct task_struct *task)
{
	if (task->ptrace & PT_SINGLESTEP)
		ptrace_cancel_bpt(task);
}

static inline void single_step_set(struct task_struct *task)
{
	if (task->ptrace & PT_SINGLESTEP)
		ptrace_set_bpt(task);
}
