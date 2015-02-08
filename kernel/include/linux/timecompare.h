
#ifndef _LINUX_TIMECOMPARE_H
#define _LINUX_TIMECOMPARE_H

#include <linux/clocksource.h>
#include <linux/ktime.h>

struct timecompare {
	struct timecounter *source;
	ktime_t (*target)(void);
	int num_samples;

	s64 offset;
	s64 skew;
	u64 last_update;
};

extern ktime_t timecompare_transform(struct timecompare *sync,
				     u64 source_tstamp);

extern int timecompare_offset(struct timecompare *sync,
			      s64 *offset,
			      u64 *source_tstamp);

extern void __timecompare_update(struct timecompare *sync,
				 u64 source_tstamp);

static inline void timecompare_update(struct timecompare *sync,
				      u64 source_tstamp)
{
	if (!source_tstamp ||
	    (s64)(source_tstamp - sync->last_update) >= NSEC_PER_SEC)
		__timecompare_update(sync, source_tstamp);
}

#endif /* _LINUX_TIMECOMPARE_H */
