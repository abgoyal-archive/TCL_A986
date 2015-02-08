
#ifndef _LINUX_PTRACE_H
#define _LINUX_PTRACE_H
/* ptrace.h */
/* structs and defines to help the user use the ptrace system call. */

/* has the defines to get at the registers. */

#define PTRACE_TRACEME		   0
#define PTRACE_PEEKTEXT		   1
#define PTRACE_PEEKDATA		   2
#define PTRACE_PEEKUSR		   3
#define PTRACE_POKETEXT		   4
#define PTRACE_POKEDATA		   5
#define PTRACE_POKEUSR		   6
#define PTRACE_CONT		   7
#define PTRACE_KILL		   8
#define PTRACE_SINGLESTEP	   9

#define PTRACE_ATTACH		  16
#define PTRACE_DETACH		  17

#define PTRACE_SYSCALL		  24

/* 0x4200-0x4300 are reserved for architecture-independent additions.  */
#define PTRACE_SETOPTIONS	0x4200
#define PTRACE_GETEVENTMSG	0x4201
#define PTRACE_GETSIGINFO	0x4202
#define PTRACE_SETSIGINFO	0x4203

#define PTRACE_GETREGSET	0x4204
#define PTRACE_SETREGSET	0x4205

/* options set using PTRACE_SETOPTIONS */
#define PTRACE_O_TRACESYSGOOD	0x00000001
#define PTRACE_O_TRACEFORK	0x00000002
#define PTRACE_O_TRACEVFORK	0x00000004
#define PTRACE_O_TRACECLONE	0x00000008
#define PTRACE_O_TRACEEXEC	0x00000010
#define PTRACE_O_TRACEVFORKDONE	0x00000020
#define PTRACE_O_TRACEEXIT	0x00000040

#define PTRACE_O_MASK		0x0000007f

/* Wait extended result codes for the above trace options.  */
#define PTRACE_EVENT_FORK	1
#define PTRACE_EVENT_VFORK	2
#define PTRACE_EVENT_CLONE	3
#define PTRACE_EVENT_EXEC	4
#define PTRACE_EVENT_VFORK_DONE	5
#define PTRACE_EVENT_EXIT	6

#include <asm/ptrace.h>

#ifdef __KERNEL__

#define PT_PTRACED	0x00000001
#define PT_DTRACE	0x00000002	/* delayed trace (used on m68k, i386) */
#define PT_TRACESYSGOOD	0x00000004
#define PT_PTRACE_CAP	0x00000008	/* ptracer can follow suid-exec */
#define PT_TRACE_FORK	0x00000010
#define PT_TRACE_VFORK	0x00000020
#define PT_TRACE_CLONE	0x00000040
#define PT_TRACE_EXEC	0x00000080
#define PT_TRACE_VFORK_DONE	0x00000100
#define PT_TRACE_EXIT	0x00000200

#define PT_TRACE_MASK	0x000003f4

/* single stepping state bits (used on ARM and PA-RISC) */
#define PT_SINGLESTEP_BIT	31
#define PT_SINGLESTEP		(1<<PT_SINGLESTEP_BIT)
#define PT_BLOCKSTEP_BIT	30
#define PT_BLOCKSTEP		(1<<PT_BLOCKSTEP_BIT)

#include <linux/compiler.h>		/* For unlikely.  */
#include <linux/sched.h>		/* For struct task_struct.  */


extern long arch_ptrace(struct task_struct *child, long request, long addr, long data);
extern int ptrace_traceme(void);
extern int ptrace_readdata(struct task_struct *tsk, unsigned long src, char __user *dst, int len);
extern int ptrace_writedata(struct task_struct *tsk, char __user *src, unsigned long dst, int len);
extern int ptrace_attach(struct task_struct *tsk);
extern int ptrace_detach(struct task_struct *, unsigned int);
extern void ptrace_disable(struct task_struct *);
extern int ptrace_check_attach(struct task_struct *task, int kill);
extern int ptrace_request(struct task_struct *child, long request, long addr, long data);
extern void ptrace_notify(int exit_code);
extern void __ptrace_link(struct task_struct *child,
			  struct task_struct *new_parent);
extern void __ptrace_unlink(struct task_struct *child);
extern void exit_ptrace(struct task_struct *tracer);
#define PTRACE_MODE_READ   1
#define PTRACE_MODE_ATTACH 2
/* Returns 0 on success, -errno on denial. */
extern int __ptrace_may_access(struct task_struct *task, unsigned int mode);
/* Returns true on success, false on denial. */
extern bool ptrace_may_access(struct task_struct *task, unsigned int mode);

static inline int ptrace_reparented(struct task_struct *child)
{
	return child->real_parent != child->parent;
}

static inline void ptrace_unlink(struct task_struct *child)
{
	if (unlikely(child->ptrace))
		__ptrace_unlink(child);
}

int generic_ptrace_peekdata(struct task_struct *tsk, long addr, long data);
int generic_ptrace_pokedata(struct task_struct *tsk, long addr, long data);

static inline int task_ptrace(struct task_struct *task)
{
	return task->ptrace;
}

static inline int ptrace_event(int mask, int event, unsigned long message)
{
	if (mask && likely(!(current->ptrace & mask)))
		return 0;
	current->ptrace_message = message;
	ptrace_notify((event << 8) | SIGTRAP);
	return 1;
}

static inline void ptrace_init_task(struct task_struct *child, bool ptrace)
{
	INIT_LIST_HEAD(&child->ptrace_entry);
	INIT_LIST_HEAD(&child->ptraced);
	child->parent = child->real_parent;
	child->ptrace = 0;
	if (unlikely(ptrace) && (current->ptrace & PT_PTRACED)) {
		child->ptrace = current->ptrace;
		__ptrace_link(child, current->parent);
	}
}

static inline void ptrace_release_task(struct task_struct *task)
{
	BUG_ON(!list_empty(&task->ptraced));
	ptrace_unlink(task);
	BUG_ON(!list_empty(&task->ptrace_entry));
}

#ifndef force_successful_syscall_return
#define force_successful_syscall_return() do { } while (0)
#endif


#ifndef arch_has_single_step
#define arch_has_single_step()		(0)

static inline void user_enable_single_step(struct task_struct *task)
{
	BUG();			/* This can never be called.  */
}

static inline void user_disable_single_step(struct task_struct *task)
{
}
#else
extern void user_enable_single_step(struct task_struct *);
extern void user_disable_single_step(struct task_struct *);
#endif	/* arch_has_single_step */

#ifndef arch_has_block_step
#define arch_has_block_step()		(0)

static inline void user_enable_block_step(struct task_struct *task)
{
	BUG();			/* This can never be called.  */
}
#else
extern void user_enable_block_step(struct task_struct *);
#endif	/* arch_has_block_step */

#ifdef ARCH_HAS_USER_SINGLE_STEP_INFO
extern void user_single_step_siginfo(struct task_struct *tsk,
				struct pt_regs *regs, siginfo_t *info);
#else
static inline void user_single_step_siginfo(struct task_struct *tsk,
				struct pt_regs *regs, siginfo_t *info)
{
	memset(info, 0, sizeof(*info));
	info->si_signo = SIGTRAP;
}
#endif

#ifndef arch_ptrace_stop_needed
#define arch_ptrace_stop_needed(code, info)	(0)
#endif

#ifndef arch_ptrace_stop
#define arch_ptrace_stop(code, info)		do { } while (0)
#endif

extern int task_current_syscall(struct task_struct *target, long *callno,
				unsigned long args[6], unsigned int maxargs,
				unsigned long *sp, unsigned long *pc);

#endif

#endif
