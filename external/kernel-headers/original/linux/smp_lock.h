#ifndef __LINUX_SMPLOCK_H
#define __LINUX_SMPLOCK_H

#ifdef CONFIG_LOCK_KERNEL
#include <linux/sched.h>
#include <linux/spinlock.h>

#define kernel_locked()		(current->lock_depth >= 0)

extern int __lockfunc __reacquire_kernel_lock(void);
extern void __lockfunc __release_kernel_lock(void);

#define release_kernel_lock(tsk) do { 		\
	if (unlikely((tsk)->lock_depth >= 0))	\
		__release_kernel_lock();	\
} while (0)

#if defined(CONFIG_SMP) && !defined(CONFIG_PREEMPT_BKL)
# define return_value_on_smp return
#else
# define return_value_on_smp
#endif

static inline int reacquire_kernel_lock(struct task_struct *task)
{
	if (unlikely(task->lock_depth >= 0))
		return_value_on_smp __reacquire_kernel_lock();
	return 0;
}

extern void __lockfunc lock_kernel(void)	__acquires(kernel_lock);
extern void __lockfunc unlock_kernel(void)	__releases(kernel_lock);

#else

#define lock_kernel()				do { } while(0)
#define unlock_kernel()				do { } while(0)
#define release_kernel_lock(task)		do { } while(0)
#define reacquire_kernel_lock(task)		0
#define kernel_locked()				1

#endif /* CONFIG_LOCK_KERNEL */
#endif /* __LINUX_SMPLOCK_H */
