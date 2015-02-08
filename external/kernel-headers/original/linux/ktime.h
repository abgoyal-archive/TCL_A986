#ifndef _LINUX_KTIME_H
#define _LINUX_KTIME_H

#include <linux/time.h>
#include <linux/jiffies.h>

typedef union {
	s64	tv64;
#if BITS_PER_LONG != 64 && !defined(CONFIG_KTIME_SCALAR)
	struct {
# ifdef __BIG_ENDIAN
	s32	sec, nsec;
# else
	s32	nsec, sec;
# endif
	} tv;
#endif
} ktime_t;

#define KTIME_MAX			((s64)~((u64)1 << 63))
#define KTIME_SEC_MAX			(KTIME_MAX / NSEC_PER_SEC)


#if (BITS_PER_LONG == 64) || defined(CONFIG_KTIME_SCALAR)

static inline ktime_t ktime_set(const long secs, const unsigned long nsecs)
{
#if (BITS_PER_LONG == 64)
	if (unlikely(secs >= KTIME_SEC_MAX))
		return (ktime_t){ .tv64 = KTIME_MAX };
#endif
	return (ktime_t) { .tv64 = (s64)secs * NSEC_PER_SEC + (s64)nsecs };
}

/* Subtract two ktime_t variables. rem = lhs -rhs: */
#define ktime_sub(lhs, rhs) \
		({ (ktime_t){ .tv64 = (lhs).tv64 - (rhs).tv64 }; })

/* Add two ktime_t variables. res = lhs + rhs: */
#define ktime_add(lhs, rhs) \
		({ (ktime_t){ .tv64 = (lhs).tv64 + (rhs).tv64 }; })

#define ktime_add_ns(kt, nsval) \
		({ (ktime_t){ .tv64 = (kt).tv64 + (nsval) }; })

/* convert a timespec to ktime_t format: */
static inline ktime_t timespec_to_ktime(struct timespec ts)
{
	return ktime_set(ts.tv_sec, ts.tv_nsec);
}

/* convert a timeval to ktime_t format: */
static inline ktime_t timeval_to_ktime(struct timeval tv)
{
	return ktime_set(tv.tv_sec, tv.tv_usec * NSEC_PER_USEC);
}

/* Map the ktime_t to timespec conversion to ns_to_timespec function */
#define ktime_to_timespec(kt)		ns_to_timespec((kt).tv64)

/* Map the ktime_t to timeval conversion to ns_to_timeval function */
#define ktime_to_timeval(kt)		ns_to_timeval((kt).tv64)

/* Convert ktime_t to nanoseconds - NOP in the scalar storage format: */
#define ktime_to_ns(kt)			((kt).tv64)

#else


/* Set a ktime_t variable to a value in sec/nsec representation: */
static inline ktime_t ktime_set(const long secs, const unsigned long nsecs)
{
	return (ktime_t) { .tv = { .sec = secs, .nsec = nsecs } };
}

static inline ktime_t ktime_sub(const ktime_t lhs, const ktime_t rhs)
{
	ktime_t res;

	res.tv64 = lhs.tv64 - rhs.tv64;
	if (res.tv.nsec < 0)
		res.tv.nsec += NSEC_PER_SEC;

	return res;
}

static inline ktime_t ktime_add(const ktime_t add1, const ktime_t add2)
{
	ktime_t res;

	res.tv64 = add1.tv64 + add2.tv64;
	/*
	 * performance trick: the (u32) -NSEC gives 0x00000000Fxxxxxxx
	 * so we subtract NSEC_PER_SEC and add 1 to the upper 32 bit.
	 *
	 * it's equivalent to:
	 *   tv.nsec -= NSEC_PER_SEC
	 *   tv.sec ++;
	 */
	if (res.tv.nsec >= NSEC_PER_SEC)
		res.tv64 += (u32)-NSEC_PER_SEC;

	return res;
}

extern ktime_t ktime_add_ns(const ktime_t kt, u64 nsec);

static inline ktime_t timespec_to_ktime(const struct timespec ts)
{
	return (ktime_t) { .tv = { .sec = (s32)ts.tv_sec,
			   	   .nsec = (s32)ts.tv_nsec } };
}

static inline ktime_t timeval_to_ktime(const struct timeval tv)
{
	return (ktime_t) { .tv = { .sec = (s32)tv.tv_sec,
				   .nsec = (s32)tv.tv_usec * 1000 } };
}

static inline struct timespec ktime_to_timespec(const ktime_t kt)
{
	return (struct timespec) { .tv_sec = (time_t) kt.tv.sec,
				   .tv_nsec = (long) kt.tv.nsec };
}

static inline struct timeval ktime_to_timeval(const ktime_t kt)
{
	return (struct timeval) {
		.tv_sec = (time_t) kt.tv.sec,
		.tv_usec = (suseconds_t) (kt.tv.nsec / NSEC_PER_USEC) };
}

static inline u64 ktime_to_ns(const ktime_t kt)
{
	return (u64) kt.tv.sec * NSEC_PER_SEC + kt.tv.nsec;
}

#endif

#define KTIME_REALTIME_RES	(ktime_t){ .tv64 = TICK_NSEC }
#define KTIME_MONOTONIC_RES	(ktime_t){ .tv64 = TICK_NSEC }

/* Get the monotonic time in timespec format: */
extern void ktime_get_ts(struct timespec *ts);

/* Get the real (wall-) time in timespec format: */
#define ktime_get_real_ts(ts)	getnstimeofday(ts)

#endif
