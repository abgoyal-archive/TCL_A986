
#ifndef _LINUX_KTIME_H
#define _LINUX_KTIME_H

#include <linux/time.h>
#include <linux/jiffies.h>

union ktime {
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
};

typedef union ktime ktime_t;		/* Kill this */

#define KTIME_MAX			((s64)~((u64)1 << 63))
#if (BITS_PER_LONG == 64)
# define KTIME_SEC_MAX			(KTIME_MAX / NSEC_PER_SEC)
#else
# define KTIME_SEC_MAX			LONG_MAX
#endif


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

#define ktime_sub_ns(kt, nsval) \
		({ (ktime_t){ .tv64 = (kt).tv64 - (nsval) }; })

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

#else	/* !((BITS_PER_LONG == 64) || defined(CONFIG_KTIME_SCALAR)) */


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

extern ktime_t ktime_sub_ns(const ktime_t kt, u64 nsec);

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

static inline s64 ktime_to_ns(const ktime_t kt)
{
	return (s64) kt.tv.sec * NSEC_PER_SEC + kt.tv.nsec;
}

#endif	/* !((BITS_PER_LONG == 64) || defined(CONFIG_KTIME_SCALAR)) */

static inline int ktime_equal(const ktime_t cmp1, const ktime_t cmp2)
{
	return cmp1.tv64 == cmp2.tv64;
}

static inline s64 ktime_to_us(const ktime_t kt)
{
	struct timeval tv = ktime_to_timeval(kt);
	return (s64) tv.tv_sec * USEC_PER_SEC + tv.tv_usec;
}

static inline s64 ktime_to_ms(const ktime_t kt)
{
	struct timeval tv = ktime_to_timeval(kt);
	return (s64) tv.tv_sec * MSEC_PER_SEC + tv.tv_usec / USEC_PER_MSEC;
}

static inline s64 ktime_us_delta(const ktime_t later, const ktime_t earlier)
{
       return ktime_to_us(ktime_sub(later, earlier));
}

static inline ktime_t ktime_add_us(const ktime_t kt, const u64 usec)
{
	return ktime_add_ns(kt, usec * 1000);
}

static inline ktime_t ktime_sub_us(const ktime_t kt, const u64 usec)
{
	return ktime_sub_ns(kt, usec * 1000);
}

extern ktime_t ktime_add_safe(const ktime_t lhs, const ktime_t rhs);

#define LOW_RES_NSEC		TICK_NSEC
#define KTIME_LOW_RES		(ktime_t){ .tv64 = LOW_RES_NSEC }

/* Get the monotonic time in timespec format: */
extern void ktime_get_ts(struct timespec *ts);

/* Get the real (wall-) time in timespec format: */
#define ktime_get_real_ts(ts)	getnstimeofday(ts)

static inline ktime_t ns_to_ktime(u64 ns)
{
	static const ktime_t ktime_zero = { .tv64 = 0 };
	return ktime_add_ns(ktime_zero, ns);
}

#endif
