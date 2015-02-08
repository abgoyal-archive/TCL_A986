

#ifdef CONFIG_DEBUG_LOCK_ALLOC

#include <linux/kernel_stat.h>

void rcu_scheduler_starting(void)
{
	WARN_ON(nr_context_switches() > 0);
	rcu_scheduler_active = 1;
}

#endif /* #ifdef CONFIG_DEBUG_LOCK_ALLOC */
