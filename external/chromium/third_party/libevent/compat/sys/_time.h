/*	$OpenBSD: time.h,v 1.11 2000/10/10 13:36:48 itojun Exp $	*/
/*	$NetBSD: time.h,v 1.18 1996/04/23 10:29:33 mycroft Exp $	*/


#ifndef _SYS_TIME_H_
#define _SYS_TIME_H_

#include <sys/types.h>

struct timeval {
	long	tv_sec;		/* seconds */
	long	tv_usec;	/* and microseconds */
};

struct timespec {
	time_t	tv_sec;		/* seconds */
	long	tv_nsec;	/* and nanoseconds */
};

#define	TIMEVAL_TO_TIMESPEC(tv, ts) {					\
	(ts)->tv_sec = (tv)->tv_sec;					\
	(ts)->tv_nsec = (tv)->tv_usec * 1000;				\
}
#define	TIMESPEC_TO_TIMEVAL(tv, ts) {					\
	(tv)->tv_sec = (ts)->tv_sec;					\
	(tv)->tv_usec = (ts)->tv_nsec / 1000;				\
}

struct timezone {
	int	tz_minuteswest;	/* minutes west of Greenwich */
	int	tz_dsttime;	/* type of dst correction */
};
#define	DST_NONE	0	/* not on dst */
#define	DST_USA		1	/* USA style dst */
#define	DST_AUST	2	/* Australian style dst */
#define	DST_WET		3	/* Western European dst */
#define	DST_MET		4	/* Middle European dst */
#define	DST_EET		5	/* Eastern European dst */
#define	DST_CAN		6	/* Canada */

/* Operations on timevals. */
#define	timerclear(tvp)		(tvp)->tv_sec = (tvp)->tv_usec = 0
#define	timerisset(tvp)		((tvp)->tv_sec || (tvp)->tv_usec)
#define	timercmp(tvp, uvp, cmp)						\
	(((tvp)->tv_sec == (uvp)->tv_sec) ?				\
	    ((tvp)->tv_usec cmp (uvp)->tv_usec) :			\
	    ((tvp)->tv_sec cmp (uvp)->tv_sec))
#define	timeradd(tvp, uvp, vvp)						\
	do {								\
		(vvp)->tv_sec = (tvp)->tv_sec + (uvp)->tv_sec;		\
		(vvp)->tv_usec = (tvp)->tv_usec + (uvp)->tv_usec;	\
		if ((vvp)->tv_usec >= 1000000) {			\
			(vvp)->tv_sec++;				\
			(vvp)->tv_usec -= 1000000;			\
		}							\
	} while (0)
#define	timersub(tvp, uvp, vvp)						\
	do {								\
		(vvp)->tv_sec = (tvp)->tv_sec - (uvp)->tv_sec;		\
		(vvp)->tv_usec = (tvp)->tv_usec - (uvp)->tv_usec;	\
		if ((vvp)->tv_usec < 0) {				\
			(vvp)->tv_sec--;				\
			(vvp)->tv_usec += 1000000;			\
		}							\
	} while (0)

/* Operations on timespecs. */
#define	timespecclear(tsp)		(tsp)->tv_sec = (tsp)->tv_nsec = 0
#define	timespecisset(tsp)		((tsp)->tv_sec || (tsp)->tv_nsec)
#define	timespeccmp(tsp, usp, cmp)					\
	(((tsp)->tv_sec == (usp)->tv_sec) ?				\
	    ((tsp)->tv_nsec cmp (usp)->tv_nsec) :			\
	    ((tsp)->tv_sec cmp (usp)->tv_sec))
#define	timespecadd(tsp, usp, vsp)					\
	do {								\
		(vsp)->tv_sec = (tsp)->tv_sec + (usp)->tv_sec;		\
		(vsp)->tv_nsec = (tsp)->tv_nsec + (usp)->tv_nsec;	\
		if ((vsp)->tv_nsec >= 1000000000L) {			\
			(vsp)->tv_sec++;				\
			(vsp)->tv_nsec -= 1000000000L;			\
		}							\
	} while (0)
#define	timespecsub(tsp, usp, vsp)					\
	do {								\
		(vsp)->tv_sec = (tsp)->tv_sec - (usp)->tv_sec;		\
		(vsp)->tv_nsec = (tsp)->tv_nsec - (usp)->tv_nsec;	\
		if ((vsp)->tv_nsec < 0) {				\
			(vsp)->tv_sec--;				\
			(vsp)->tv_nsec += 1000000000L;			\
		}							\
	} while (0)

#define	ITIMER_REAL	0
#define	ITIMER_VIRTUAL	1
#define	ITIMER_PROF	2

struct	itimerval {
	struct	timeval it_interval;	/* timer interval */
	struct	timeval it_value;	/* current value */
};

struct clockinfo {
	int	hz;		/* clock frequency */
	int	tick;		/* micro-seconds per hz tick */
	int	tickadj;	/* clock skew rate for adjtime() */
	int	stathz;		/* statistics clock frequency */
	int	profhz;		/* profiling clock frequency */
};

#define CLOCK_REALTIME	0
#define CLOCK_VIRTUAL	1
#define CLOCK_PROF	2

#define TIMER_RELTIME	0x0	/* relative timer */
#define TIMER_ABSTIME	0x1	/* absolute timer */

/* --- stuff got cut here - niels --- */

#endif /* !_SYS_TIME_H_ */
