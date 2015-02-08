
#ifndef __RES_COUNTER_H__
#define __RES_COUNTER_H__


#include <linux/cgroup.h>


struct res_counter {
	/*
	 * the current resource consumption level
	 */
	unsigned long long usage;
	/*
	 * the maximal value of the usage from the counter creation
	 */
	unsigned long long max_usage;
	/*
	 * the limit that usage cannot exceed
	 */
	unsigned long long limit;
	/*
	 * the limit that usage can be exceed
	 */
	unsigned long long soft_limit;
	/*
	 * the number of unsuccessful attempts to consume the resource
	 */
	unsigned long long failcnt;
	/*
	 * the lock to protect all of the above.
	 * the routines below consider this to be IRQ-safe
	 */
	spinlock_t lock;
	/*
	 * Parent counter, used for hierarchial resource accounting
	 */
	struct res_counter *parent;
};

#define RESOURCE_MAX (unsigned long long)LLONG_MAX


u64 res_counter_read_u64(struct res_counter *counter, int member);

ssize_t res_counter_read(struct res_counter *counter, int member,
		const char __user *buf, size_t nbytes, loff_t *pos,
		int (*read_strategy)(unsigned long long val, char *s));

typedef int (*write_strategy_fn)(const char *buf, unsigned long long *val);

int res_counter_memparse_write_strategy(const char *buf,
					unsigned long long *res);

int res_counter_write(struct res_counter *counter, int member,
		      const char *buffer, write_strategy_fn write_strategy);


enum {
	RES_USAGE,
	RES_MAX_USAGE,
	RES_LIMIT,
	RES_FAILCNT,
	RES_SOFT_LIMIT,
};


void res_counter_init(struct res_counter *counter, struct res_counter *parent);


int __must_check res_counter_charge_locked(struct res_counter *counter,
		unsigned long val);
int __must_check res_counter_charge(struct res_counter *counter,
		unsigned long val, struct res_counter **limit_fail_at);


void res_counter_uncharge_locked(struct res_counter *counter, unsigned long val);
void res_counter_uncharge(struct res_counter *counter, unsigned long val);

static inline bool res_counter_limit_check_locked(struct res_counter *cnt)
{
	if (cnt->usage < cnt->limit)
		return true;

	return false;
}

static inline bool res_counter_soft_limit_check_locked(struct res_counter *cnt)
{
	if (cnt->usage < cnt->soft_limit)
		return true;

	return false;
}

static inline unsigned long long
res_counter_soft_limit_excess(struct res_counter *cnt)
{
	unsigned long long excess;
	unsigned long flags;

	spin_lock_irqsave(&cnt->lock, flags);
	if (cnt->usage <= cnt->soft_limit)
		excess = 0;
	else
		excess = cnt->usage - cnt->soft_limit;
	spin_unlock_irqrestore(&cnt->lock, flags);
	return excess;
}

static inline bool res_counter_check_under_limit(struct res_counter *cnt)
{
	bool ret;
	unsigned long flags;

	spin_lock_irqsave(&cnt->lock, flags);
	ret = res_counter_limit_check_locked(cnt);
	spin_unlock_irqrestore(&cnt->lock, flags);
	return ret;
}

static inline bool res_counter_check_under_soft_limit(struct res_counter *cnt)
{
	bool ret;
	unsigned long flags;

	spin_lock_irqsave(&cnt->lock, flags);
	ret = res_counter_soft_limit_check_locked(cnt);
	spin_unlock_irqrestore(&cnt->lock, flags);
	return ret;
}

static inline void res_counter_reset_max(struct res_counter *cnt)
{
	unsigned long flags;

	spin_lock_irqsave(&cnt->lock, flags);
	cnt->max_usage = cnt->usage;
	spin_unlock_irqrestore(&cnt->lock, flags);
}

static inline void res_counter_reset_failcnt(struct res_counter *cnt)
{
	unsigned long flags;

	spin_lock_irqsave(&cnt->lock, flags);
	cnt->failcnt = 0;
	spin_unlock_irqrestore(&cnt->lock, flags);
}

static inline int res_counter_set_limit(struct res_counter *cnt,
		unsigned long long limit)
{
	unsigned long flags;
	int ret = -EBUSY;

	spin_lock_irqsave(&cnt->lock, flags);
	if (cnt->usage <= limit) {
		cnt->limit = limit;
		ret = 0;
	}
	spin_unlock_irqrestore(&cnt->lock, flags);
	return ret;
}

static inline int
res_counter_set_soft_limit(struct res_counter *cnt,
				unsigned long long soft_limit)
{
	unsigned long flags;

	spin_lock_irqsave(&cnt->lock, flags);
	cnt->soft_limit = soft_limit;
	spin_unlock_irqrestore(&cnt->lock, flags);
	return 0;
}

#endif
