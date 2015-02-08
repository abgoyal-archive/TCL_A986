
/* Freezer declarations */

#ifndef FREEZER_H_INCLUDED
#define FREEZER_H_INCLUDED

#include <linux/sched.h>
#include <linux/wait.h>

#ifdef CONFIG_FREEZER
static inline int frozen(struct task_struct *p)
{
	return p->flags & PF_FROZEN;
}

static inline int freezing(struct task_struct *p)
{
	return test_tsk_thread_flag(p, TIF_FREEZE);
}

static inline void set_freeze_flag(struct task_struct *p)
{
	set_tsk_thread_flag(p, TIF_FREEZE);
}

static inline void clear_freeze_flag(struct task_struct *p)
{
	clear_tsk_thread_flag(p, TIF_FREEZE);
}

static inline bool should_send_signal(struct task_struct *p)
{
	return !(p->flags & PF_FREEZER_NOSIG);
}

/* Takes and releases task alloc lock using task_lock() */
extern int thaw_process(struct task_struct *p);

extern void refrigerator(void);
extern int freeze_processes(void);
extern void thaw_processes(void);

static inline int try_to_freeze(void)
{
	if (freezing(current)) {
		refrigerator();
		return 1;
	} else
		return 0;
}

extern bool freeze_task(struct task_struct *p, bool sig_only);
extern void cancel_freezing(struct task_struct *p);

#ifdef CONFIG_CGROUP_FREEZER
extern int cgroup_freezing_or_frozen(struct task_struct *task);
#else /* !CONFIG_CGROUP_FREEZER */
static inline int cgroup_freezing_or_frozen(struct task_struct *task)
{
	return 0;
}
#endif /* !CONFIG_CGROUP_FREEZER */


static inline void freezer_do_not_count(void)
{
	if (current->mm)
		current->flags |= PF_FREEZER_SKIP;
}

static inline void freezer_count(void)
{
	if (current->mm) {
		current->flags &= ~PF_FREEZER_SKIP;
		try_to_freeze();
	}
}

static inline int freezer_should_skip(struct task_struct *p)
{
	return !!(p->flags & PF_FREEZER_SKIP);
}

static inline void set_freezable(void)
{
	current->flags &= ~PF_NOFREEZE;
}

static inline void set_freezable_with_signal(void)
{
	current->flags &= ~(PF_NOFREEZE | PF_FREEZER_NOSIG);
}


#define wait_event_freezable(wq, condition)				\
({									\
	int __retval;							\
	do {								\
		__retval = wait_event_interruptible(wq, 		\
				(condition) || freezing(current));	\
		if (__retval && !freezing(current))			\
			break;						\
		else if (!(condition))					\
			__retval = -ERESTARTSYS;			\
	} while (try_to_freeze());					\
	__retval;							\
})


#define wait_event_freezable_timeout(wq, condition, timeout)		\
({									\
	long __retval = timeout;					\
	do {								\
		__retval = wait_event_interruptible_timeout(wq,		\
				(condition) || freezing(current),	\
				__retval); 				\
	} while (try_to_freeze());					\
	__retval;							\
})
#else /* !CONFIG_FREEZER */
static inline int frozen(struct task_struct *p) { return 0; }
static inline int freezing(struct task_struct *p) { return 0; }
static inline void set_freeze_flag(struct task_struct *p) {}
static inline void clear_freeze_flag(struct task_struct *p) {}
static inline int thaw_process(struct task_struct *p) { return 1; }

static inline void refrigerator(void) {}
static inline int freeze_processes(void) { BUG(); return 0; }
static inline void thaw_processes(void) {}

static inline int try_to_freeze(void) { return 0; }

static inline void freezer_do_not_count(void) {}
static inline void freezer_count(void) {}
static inline int freezer_should_skip(struct task_struct *p) { return 0; }
static inline void set_freezable(void) {}
static inline void set_freezable_with_signal(void) {}

#define wait_event_freezable(wq, condition)				\
		wait_event_interruptible(wq, condition)

#define wait_event_freezable_timeout(wq, condition, timeout)		\
		wait_event_interruptible_timeout(wq, condition, timeout)

#endif /* !CONFIG_FREEZER */

#endif	/* FREEZER_H_INCLUDED */
