#ifndef _LINUX_JIFFIES_H
#define _LINUX_JIFFIES_H

#include <linux/calc64.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <asm/param.h>			/* for HZ */

#if HZ >= 12 && HZ < 24
# define SHIFT_HZ	4
#elif HZ >= 24 && HZ < 48
# define SHIFT_HZ	5
#elif HZ >= 48 && HZ < 96
# define SHIFT_HZ	6
#elif HZ >= 96 && HZ < 192
# define SHIFT_HZ	7
#elif HZ >= 192 && HZ < 384
# define SHIFT_HZ	8
#elif HZ >= 384 && HZ < 768
# define SHIFT_HZ	9
#elif HZ >= 768 && HZ < 1536
# define SHIFT_HZ	10
#else
# error You lose.
#endif

/* LATCH is used in the interval timer and ftape setup. */
#define LATCH  ((CLOCK_TICK_RATE + HZ/2) / HZ)	/* For divider */

#define LATCH_HPET ((HPET_TICK_RATE + HZ/2) / HZ)

#define SH_DIV(NOM,DEN,LSH) (   (((NOM) / (DEN)) << (LSH))              \
                             + ((((NOM) % (DEN)) << (LSH)) + (DEN) / 2) / (DEN))

/* HZ is the requested value. ACTHZ is actual HZ ("<< 8" is for accuracy) */
#define ACTHZ (SH_DIV (CLOCK_TICK_RATE, LATCH, 8))

#define ACTHZ_HPET (SH_DIV (HPET_TICK_RATE, LATCH_HPET, 8))

/* TICK_NSEC is the time between ticks in nsec assuming real ACTHZ */
#define TICK_NSEC (SH_DIV (1000000UL * 1000, ACTHZ, 8))

#define TICK_NSEC_HPET (SH_DIV(1000000UL * 1000, ACTHZ_HPET, 8))

/* TICK_USEC is the time between ticks in usec assuming fake USER_HZ */
#define TICK_USEC ((1000000UL + USER_HZ/2) / USER_HZ)

/* TICK_USEC_TO_NSEC is the time between ticks in nsec assuming real ACTHZ and	*/
/* a value TUSEC for TICK_USEC (can be set bij adjtimex)		*/
#define TICK_USEC_TO_NSEC(TUSEC) (SH_DIV (TUSEC * USER_HZ * 1000, ACTHZ, 8))

#define __jiffy_data  __attribute__((section(".data")))

extern u64 __jiffy_data jiffies_64;
extern unsigned long volatile __jiffy_data jiffies;

#if (BITS_PER_LONG < 64)
u64 get_jiffies_64(void);
#else
static inline u64 get_jiffies_64(void)
{
	return (u64)jiffies;
}
#endif

#define time_after(a,b)		\
	(typecheck(unsigned long, a) && \
	 typecheck(unsigned long, b) && \
	 ((long)(b) - (long)(a) < 0))
#define time_before(a,b)	time_after(b,a)

#define time_after_eq(a,b)	\
	(typecheck(unsigned long, a) && \
	 typecheck(unsigned long, b) && \
	 ((long)(a) - (long)(b) >= 0))
#define time_before_eq(a,b)	time_after_eq(b,a)

#define INITIAL_JIFFIES ((unsigned long)(unsigned int) (-300*HZ))

#define MAX_JIFFY_OFFSET ((~0UL >> 1)-1)



#define SEC_JIFFIE_SC (31 - SHIFT_HZ)
#if !((((NSEC_PER_SEC << 2) / TICK_NSEC) << (SEC_JIFFIE_SC - 2)) & 0x80000000)
#undef SEC_JIFFIE_SC
#define SEC_JIFFIE_SC (32 - SHIFT_HZ)
#endif
#define NSEC_JIFFIE_SC (SEC_JIFFIE_SC + 29)
#define USEC_JIFFIE_SC (SEC_JIFFIE_SC + 19)
#define SEC_CONVERSION ((unsigned long)((((u64)NSEC_PER_SEC << SEC_JIFFIE_SC) +\
                                TICK_NSEC -1) / (u64)TICK_NSEC))

#define NSEC_CONVERSION ((unsigned long)((((u64)1 << NSEC_JIFFIE_SC) +\
                                        TICK_NSEC -1) / (u64)TICK_NSEC))
#define USEC_CONVERSION  \
                    ((unsigned long)((((u64)NSEC_PER_USEC << USEC_JIFFIE_SC) +\
                                        TICK_NSEC -1) / (u64)TICK_NSEC))
#define USEC_ROUND (u64)(((u64)1 << USEC_JIFFIE_SC) - 1)
#if BITS_PER_LONG < 64
# define MAX_SEC_IN_JIFFIES \
	(long)((u64)((u64)MAX_JIFFY_OFFSET * TICK_NSEC) / NSEC_PER_SEC)
#else	/* take care of overflow on 64 bits machines */
# define MAX_SEC_IN_JIFFIES \
	(SH_DIV((MAX_JIFFY_OFFSET >> SEC_JIFFIE_SC) * TICK_NSEC, NSEC_PER_SEC, 1) - 1)

#endif

static inline unsigned int jiffies_to_msecs(const unsigned long j)
{
#if HZ <= MSEC_PER_SEC && !(MSEC_PER_SEC % HZ)
	return (MSEC_PER_SEC / HZ) * j;
#elif HZ > MSEC_PER_SEC && !(HZ % MSEC_PER_SEC)
	return (j + (HZ / MSEC_PER_SEC) - 1)/(HZ / MSEC_PER_SEC);
#else
	return (j * MSEC_PER_SEC) / HZ;
#endif
}

static inline unsigned int jiffies_to_usecs(const unsigned long j)
{
#if HZ <= USEC_PER_SEC && !(USEC_PER_SEC % HZ)
	return (USEC_PER_SEC / HZ) * j;
#elif HZ > USEC_PER_SEC && !(HZ % USEC_PER_SEC)
	return (j + (HZ / USEC_PER_SEC) - 1)/(HZ / USEC_PER_SEC);
#else
	return (j * USEC_PER_SEC) / HZ;
#endif
}

static inline unsigned long msecs_to_jiffies(const unsigned int m)
{
	if (m > jiffies_to_msecs(MAX_JIFFY_OFFSET))
		return MAX_JIFFY_OFFSET;
#if HZ <= MSEC_PER_SEC && !(MSEC_PER_SEC % HZ)
	return (m + (MSEC_PER_SEC / HZ) - 1) / (MSEC_PER_SEC / HZ);
#elif HZ > MSEC_PER_SEC && !(HZ % MSEC_PER_SEC)
	return m * (HZ / MSEC_PER_SEC);
#else
	return (m * HZ + MSEC_PER_SEC - 1) / MSEC_PER_SEC;
#endif
}

static inline unsigned long usecs_to_jiffies(const unsigned int u)
{
	if (u > jiffies_to_usecs(MAX_JIFFY_OFFSET))
		return MAX_JIFFY_OFFSET;
#if HZ <= USEC_PER_SEC && !(USEC_PER_SEC % HZ)
	return (u + (USEC_PER_SEC / HZ) - 1) / (USEC_PER_SEC / HZ);
#elif HZ > USEC_PER_SEC && !(HZ % USEC_PER_SEC)
	return u * (HZ / USEC_PER_SEC);
#else
	return (u * HZ + USEC_PER_SEC - 1) / USEC_PER_SEC;
#endif
}

static __inline__ unsigned long
timespec_to_jiffies(const struct timespec *value)
{
	unsigned long sec = value->tv_sec;
	long nsec = value->tv_nsec + TICK_NSEC - 1;

	if (sec >= MAX_SEC_IN_JIFFIES){
		sec = MAX_SEC_IN_JIFFIES;
		nsec = 0;
	}
	return (((u64)sec * SEC_CONVERSION) +
		(((u64)nsec * NSEC_CONVERSION) >>
		 (NSEC_JIFFIE_SC - SEC_JIFFIE_SC))) >> SEC_JIFFIE_SC;

}

static __inline__ void
jiffies_to_timespec(const unsigned long jiffies, struct timespec *value)
{
	/*
	 * Convert jiffies to nanoseconds and separate with
	 * one divide.
	 */
	u64 nsec = (u64)jiffies * TICK_NSEC;
	value->tv_sec = div_long_long_rem(nsec, NSEC_PER_SEC, &value->tv_nsec);
}

static __inline__ unsigned long
timeval_to_jiffies(const struct timeval *value)
{
	unsigned long sec = value->tv_sec;
	long usec = value->tv_usec;

	if (sec >= MAX_SEC_IN_JIFFIES){
		sec = MAX_SEC_IN_JIFFIES;
		usec = 0;
	}
	return (((u64)sec * SEC_CONVERSION) +
		(((u64)usec * USEC_CONVERSION + USEC_ROUND) >>
		 (USEC_JIFFIE_SC - SEC_JIFFIE_SC))) >> SEC_JIFFIE_SC;
}

static __inline__ void
jiffies_to_timeval(const unsigned long jiffies, struct timeval *value)
{
	/*
	 * Convert jiffies to nanoseconds and separate with
	 * one divide.
	 */
	u64 nsec = (u64)jiffies * TICK_NSEC;
	long tv_usec;

	value->tv_sec = div_long_long_rem(nsec, NSEC_PER_SEC, &tv_usec);
	tv_usec /= NSEC_PER_USEC;
	value->tv_usec = tv_usec;
}

static inline clock_t jiffies_to_clock_t(long x)
{
#if (TICK_NSEC % (NSEC_PER_SEC / USER_HZ)) == 0
	return x / (HZ / USER_HZ);
#else
	u64 tmp = (u64)x * TICK_NSEC;
	do_div(tmp, (NSEC_PER_SEC / USER_HZ));
	return (long)tmp;
#endif
}

static inline unsigned long clock_t_to_jiffies(unsigned long x)
{
#if (HZ % USER_HZ)==0
	if (x >= ~0UL / (HZ / USER_HZ))
		return ~0UL;
	return x * (HZ / USER_HZ);
#else
	u64 jif;

	/* Don't worry about loss of precision here .. */
	if (x >= ~0UL / HZ * USER_HZ)
		return ~0UL;

	/* .. but do try to contain it here */
	jif = x * (u64) HZ;
	do_div(jif, USER_HZ);
	return jif;
#endif
}

static inline u64 jiffies_64_to_clock_t(u64 x)
{
#if (TICK_NSEC % (NSEC_PER_SEC / USER_HZ)) == 0
	do_div(x, HZ / USER_HZ);
#else
	/*
	 * There are better ways that don't overflow early,
	 * but even this doesn't overflow in hundreds of years
	 * in 64 bits, so..
	 */
	x *= TICK_NSEC;
	do_div(x, (NSEC_PER_SEC / USER_HZ));
#endif
	return x;
}

static inline u64 nsec_to_clock_t(u64 x)
{
#if (NSEC_PER_SEC % USER_HZ) == 0
	do_div(x, (NSEC_PER_SEC / USER_HZ));
#elif (USER_HZ % 512) == 0
	x *= USER_HZ/512;
	do_div(x, (NSEC_PER_SEC / 512));
#else
	/*
         * max relative error 5.7e-8 (1.8s per year) for USER_HZ <= 1024,
         * overflow after 64.99 years.
         * exact for HZ=60, 72, 90, 120, 144, 180, 300, 600, 900, ...
         */
	x *= 9;
	do_div(x, (unsigned long)((9ull * NSEC_PER_SEC + (USER_HZ/2))
	                          / USER_HZ));
#endif
	return x;
}

#endif
