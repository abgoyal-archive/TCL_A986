

#ifndef _ASM_SYSCALL_H
#define _ASM_SYSCALL_H

#include <linux/err.h>
#include <asm/ptrace.h>

static inline long syscall_get_nr(struct task_struct *task,
				  struct pt_regs *regs)
{
	return regs->syscallno;
}

static inline void syscall_rollback(struct task_struct *task,
				    struct pt_regs *regs)
{
	regs->gr8 = regs->orig_gr8;
}

static inline long syscall_get_error(struct task_struct *task,
				     struct pt_regs *regs)
{
	return IS_ERR_VALUE(regs->gr8) ? regs->gr8 : 0;
}

static inline long syscall_get_return_value(struct task_struct *task,
					    struct pt_regs *regs)
{
	return regs->gr8;
}

static inline void syscall_set_return_value(struct task_struct *task,
					    struct pt_regs *regs,
					    int error, long val)
{
	if (error)
		regs->gr8 = -error;
	else
		regs->gr8 = val;
}

static inline void syscall_get_arguments(struct task_struct *task,
					 struct pt_regs *regs,
					 unsigned int i, unsigned int n,
					 unsigned long *args)
{
	/*
	 * Do this simply for now. If we need to start supporting
	 * fetching arguments from arbitrary indices, this will need some
	 * extra logic. Presently there are no in-tree users that depend
	 * on this behaviour.
	 */
	BUG_ON(i);

	/* Argument pattern is: GR8, GR9, GR10, GR11, GR12, GR13 */
	switch (n) {
	case 6: args[5] = regs->gr13;
	case 5: args[4] = regs->gr12;
	case 4: args[3] = regs->gr11;
	case 3: args[2] = regs->gr10;
	case 2: args[1] = regs->gr9;
	case 1:	args[0] = regs->gr8;
		break;
	default:
		BUG();
	}
}

static inline void syscall_set_arguments(struct task_struct *task,
					 struct pt_regs *regs,
					 unsigned int i, unsigned int n,
					 const unsigned long *args)
{
	/* Same note as above applies */
	BUG_ON(i);

	switch (n) {
	case 6: regs->gr13 = args[5];
	case 5: regs->gr12 = args[4];
	case 4: regs->gr11 = args[3];
	case 3: regs->gr10 = args[2];
	case 2: regs->gr9  = args[1];
	case 1: regs->gr8  = args[0];
		break;
	default:
		BUG();
	}
}

#endif /* _ASM_SYSCALL_H */
