
#ifndef _SPARC_CURRENT_H
#define _SPARC_CURRENT_H

#include <linux/thread_info.h>

#ifdef CONFIG_SPARC64
register struct task_struct *current asm("g4");
#endif

#ifdef CONFIG_SPARC32
struct task_struct;
static inline struct task_struct *__get_current(void)
{
	return current_thread_info()->task;
}
#define current __get_current()
#endif

#endif /* !(_SPARC_CURRENT_H) */
