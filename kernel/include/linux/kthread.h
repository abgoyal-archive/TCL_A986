
#ifndef _LINUX_KTHREAD_H
#define _LINUX_KTHREAD_H
/* Simple interface for creating and stopping kernel threads without mess. */
#include <linux/err.h>
#include <linux/sched.h>

struct task_struct *kthread_create(int (*threadfn)(void *data),
				   void *data,
				   const char namefmt[], ...)
	__attribute__((format(printf, 3, 4)));

#define kthread_run(threadfn, data, namefmt, ...)			   \
({									   \
	struct task_struct *__k						   \
		= kthread_create(threadfn, data, namefmt, ## __VA_ARGS__); \
	if (!IS_ERR(__k))						   \
		wake_up_process(__k);					   \
	__k;								   \
})

void kthread_bind(struct task_struct *k, unsigned int cpu);
int kthread_stop(struct task_struct *k);
int kthread_should_stop(void);

int kthreadd(void *unused);
extern struct task_struct *kthreadd_task;

#endif /* _LINUX_KTHREAD_H */
