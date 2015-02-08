

#ifndef _LINUX_KMEMTRACE_H
#define _LINUX_KMEMTRACE_H

#ifdef __KERNEL__

#include <trace/events/kmem.h>

#ifdef CONFIG_KMEMTRACE
extern void kmemtrace_init(void);
#else
static inline void kmemtrace_init(void)
{
}
#endif

#endif /* __KERNEL__ */

#endif /* _LINUX_KMEMTRACE_H */

