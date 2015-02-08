
#ifndef _ASM_CURRENT_H
#define _ASM_CURRENT_H

#include <linux/thread_info.h>

#ifdef CONFIG_MN10300_CURRENT_IN_E2

register struct task_struct *const current asm("e2") __attribute__((used));

#define get_current() current

extern struct task_struct *__current;

#else
static inline __attribute__((const))
struct task_struct *get_current(void)
{
	return current_thread_info()->task;
}

#define current get_current()
#endif

#endif /* _ASM_CURRENT_H */
