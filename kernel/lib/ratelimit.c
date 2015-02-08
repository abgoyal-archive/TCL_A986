

#include <linux/ratelimit.h>
#include <linux/jiffies.h>
#include <linux/module.h>

int ___ratelimit(struct ratelimit_state *rs, const char *func)
{
	unsigned long flags;
	int ret;

	if (!rs->interval)
		return 1;

	/*
	 * If we contend on this state's lock then almost
	 * by definition we are too busy to print a message,
	 * in addition to the one that will be printed by
	 * the entity that is holding the lock already:
	 */
	if (!spin_trylock_irqsave(&rs->lock, flags))
		return 0;

	if (!rs->begin)
		rs->begin = jiffies;

	if (time_is_before_jiffies(rs->begin + rs->interval)) {
		if (rs->missed)
			printk(KERN_WARNING "%s: %d callbacks suppressed\n",
				func, rs->missed);
		rs->begin   = 0;
		rs->printed = 0;
		rs->missed  = 0;
	}
	if (rs->burst && rs->burst > rs->printed) {
		rs->printed++;
		ret = 1;
	} else {
		rs->missed++;
		ret = 0;
	}
	spin_unlock_irqrestore(&rs->lock, flags);

	return ret;
}
EXPORT_SYMBOL(___ratelimit);
