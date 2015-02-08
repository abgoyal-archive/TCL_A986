
#include <linux/module.h>
#include <linux/kallsyms.h>
#include <linux/semaphore.h>
#include <linux/smp_lock.h>

#define CREATE_TRACE_POINTS
#include <trace/events/bkl.h>

static  __cacheline_aligned_in_smp DEFINE_RAW_SPINLOCK(kernel_flag);


int __lockfunc __reacquire_kernel_lock(void)
{
	while (!do_raw_spin_trylock(&kernel_flag)) {
		if (need_resched())
			return -EAGAIN;
		cpu_relax();
	}
	preempt_disable();
	return 0;
}

void __lockfunc __release_kernel_lock(void)
{
	do_raw_spin_unlock(&kernel_flag);
	preempt_enable_no_resched();
}

#ifdef CONFIG_PREEMPT
static inline void __lock_kernel(void)
{
	preempt_disable();
	if (unlikely(!do_raw_spin_trylock(&kernel_flag))) {
		/*
		 * If preemption was disabled even before this
		 * was called, there's nothing we can be polite
		 * about - just spin.
		 */
		if (preempt_count() > 1) {
			do_raw_spin_lock(&kernel_flag);
			return;
		}

		/*
		 * Otherwise, let's wait for the kernel lock
		 * with preemption enabled..
		 */
		do {
			preempt_enable();
			while (raw_spin_is_locked(&kernel_flag))
				cpu_relax();
			preempt_disable();
		} while (!do_raw_spin_trylock(&kernel_flag));
	}
}

#else

static inline void __lock_kernel(void)
{
	do_raw_spin_lock(&kernel_flag);
}
#endif

static inline void __unlock_kernel(void)
{
	/*
	 * the BKL is not covered by lockdep, so we open-code the
	 * unlocking sequence (and thus avoid the dep-chain ops):
	 */
	do_raw_spin_unlock(&kernel_flag);
	preempt_enable();
}

void __lockfunc _lock_kernel(const char *func, const char *file, int line)
{
	int depth = current->lock_depth + 1;

	trace_lock_kernel(func, file, line);

	if (likely(!depth)) {
		might_sleep();
		__lock_kernel();
	}
	current->lock_depth = depth;
}

void __lockfunc _unlock_kernel(const char *func, const char *file, int line)
{
	BUG_ON(current->lock_depth < 0);
	if (likely(--current->lock_depth < 0))
		__unlock_kernel();

	trace_unlock_kernel(func, file, line);
}

EXPORT_SYMBOL(_lock_kernel);
EXPORT_SYMBOL(_unlock_kernel);

