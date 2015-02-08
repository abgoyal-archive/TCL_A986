
#ifndef _LINUX_TIMEX_H
#define _LINUX_TIMEX_H

#include <linux/compiler.h>
#include <linux/time.h>

#include <asm/param.h>

#define SHIFT_KG 6		/* phase factor (shift) */
#define SHIFT_KF 16		/* PLL frequency factor (shift) */
#define SHIFT_KH 2		/* FLL frequency factor (shift) */
#define MAXTC 6			/* maximum time constant (shift) */

#define SHIFT_SCALE 22		/* phase scale (shift) */
#define SHIFT_UPDATE (SHIFT_KG + MAXTC) /* time offset scale (shift) */
#define SHIFT_USEC 16		/* frequency offset scale (shift) */
#define FINENSEC (1L << (SHIFT_SCALE - 10)) /* ~1 ns in phase units */

#define MAXPHASE 512000L        /* max phase error (us) */
#define MAXFREQ (512L << SHIFT_USEC)  /* max frequency error (ppm) */
#define MINSEC 16L              /* min interval between updates (s) */
#define MAXSEC 1200L            /* max interval between updates (s) */
#define	NTP_PHASE_LIMIT	(MAXPHASE << 5)	/* beyond max. dispersion */

struct timex {
	unsigned int modes;	/* mode selector */
	long offset;		/* time offset (usec) */
	long freq;		/* frequency offset (scaled ppm) */
	long maxerror;		/* maximum error (usec) */
	long esterror;		/* estimated error (usec) */
	int status;		/* clock command/status */
	long constant;		/* pll time constant */
	long precision;		/* clock precision (usec) (read only) */
	long tolerance;		/* clock frequency tolerance (ppm)
				 * (read only)
				 */
	struct timeval time;	/* (read only) */
	long tick;		/* (modified) usecs between clock ticks */

	long ppsfreq;           /* pps frequency (scaled ppm) (ro) */
	long jitter;            /* pps jitter (us) (ro) */
	int shift;              /* interval duration (s) (shift) (ro) */
	long stabil;            /* pps stability (scaled ppm) (ro) */
	long jitcnt;            /* jitter limit exceeded (ro) */
	long calcnt;            /* calibration intervals (ro) */
	long errcnt;            /* calibration errors (ro) */
	long stbcnt;            /* stability limit exceeded (ro) */

	int  :32; int  :32; int  :32; int  :32;
	int  :32; int  :32; int  :32; int  :32;
	int  :32; int  :32; int  :32; int  :32;
};

#define ADJ_OFFSET		0x0001	/* time offset */
#define ADJ_FREQUENCY		0x0002	/* frequency offset */
#define ADJ_MAXERROR		0x0004	/* maximum time error */
#define ADJ_ESTERROR		0x0008	/* estimated time error */
#define ADJ_STATUS		0x0010	/* clock status */
#define ADJ_TIMECONST		0x0020	/* pll time constant */
#define ADJ_TICK		0x4000	/* tick value */
#define ADJ_OFFSET_SINGLESHOT	0x8001	/* old-fashioned adjtime */

/* xntp 3.4 compatibility names */
#define MOD_OFFSET	ADJ_OFFSET
#define MOD_FREQUENCY	ADJ_FREQUENCY
#define MOD_MAXERROR	ADJ_MAXERROR
#define MOD_ESTERROR	ADJ_ESTERROR
#define MOD_STATUS	ADJ_STATUS
#define MOD_TIMECONST	ADJ_TIMECONST
#define MOD_CLKB	ADJ_TICK
#define MOD_CLKA	ADJ_OFFSET_SINGLESHOT /* 0x8000 in original */


#define STA_PLL		0x0001	/* enable PLL updates (rw) */
#define STA_PPSFREQ	0x0002	/* enable PPS freq discipline (rw) */
#define STA_PPSTIME	0x0004	/* enable PPS time discipline (rw) */
#define STA_FLL		0x0008	/* select frequency-lock mode (rw) */

#define STA_INS		0x0010	/* insert leap (rw) */
#define STA_DEL		0x0020	/* delete leap (rw) */
#define STA_UNSYNC	0x0040	/* clock unsynchronized (rw) */
#define STA_FREQHOLD	0x0080	/* hold frequency (rw) */

#define STA_PPSSIGNAL	0x0100	/* PPS signal present (ro) */
#define STA_PPSJITTER	0x0200	/* PPS signal jitter exceeded (ro) */
#define STA_PPSWANDER	0x0400	/* PPS signal wander exceeded (ro) */
#define STA_PPSERROR	0x0800	/* PPS signal calibration error (ro) */

#define STA_CLOCKERR	0x1000	/* clock hardware fault (ro) */

#define STA_RONLY (STA_PPSSIGNAL | STA_PPSJITTER | STA_PPSWANDER | \
    STA_PPSERROR | STA_CLOCKERR) /* read-only bits */

#define TIME_OK		0	/* clock synchronized, no leap second */
#define TIME_INS	1	/* insert leap second */
#define TIME_DEL	2	/* delete leap second */
#define TIME_OOP	3	/* leap second in progress */
#define TIME_WAIT	4	/* leap second has occurred */
#define TIME_ERROR	5	/* clock not synchronized */
#define TIME_BAD	TIME_ERROR /* bw compat */

#ifdef __KERNEL__
#include <asm/timex.h>

extern unsigned long tick_usec;		/* USER_HZ period (usec) */
extern unsigned long tick_nsec;		/* ACTHZ          period (nsec) */
extern int tickadj;			/* amount of adjustment per tick */

extern int time_state;		/* clock status */
extern int time_status;		/* clock synchronization status bits */
extern long time_offset;	/* time adjustment (us) */
extern long time_constant;	/* pll time constant */
extern long time_tolerance;	/* frequency tolerance (ppm) */
extern long time_precision;	/* clock precision (us) */
extern long time_maxerror;	/* maximum error */
extern long time_esterror;	/* estimated error */

extern long time_freq;		/* frequency offset (scaled ppm) */
extern long time_reftime;	/* time at last adjustment (s) */

extern long time_adjust;	/* The amount of adjtime left */
extern long time_next_adjust;	/* Value for time_adjust at next tick */

static inline void ntp_clear(void)
{
	time_adjust = 0;		/* stop active adjtime() */
	time_status |= STA_UNSYNC;
	time_maxerror = NTP_PHASE_LIMIT;
	time_esterror = NTP_PHASE_LIMIT;
}

static inline int ntp_synced(void)
{
	return !(time_status & STA_UNSYNC);
}

/* Required to safely shift negative values */
#define shift_right(x, s) ({	\
	__typeof__(x) __x = (x);	\
	__typeof__(s) __s = (s);	\
	__x < 0 ? -(-__x >> __s) : __x >> __s;	\
})


#ifdef CONFIG_TIME_INTERPOLATION

#define TIME_SOURCE_CPU 0
#define TIME_SOURCE_MMIO64 1
#define TIME_SOURCE_MMIO32 2
#define TIME_SOURCE_FUNCTION 3


struct time_interpolator {
	u16 source;			/* time source flags */
	u8 shift;			/* increases accuracy of multiply by shifting. */
				/* Note that bits may be lost if shift is set too high */
	u8 jitter;			/* if set compensate for fluctuations */
	u32 nsec_per_cyc;		/* set by register_time_interpolator() */
	void *addr;			/* address of counter or function */
	u64 mask;			/* mask the valid bits of the counter */
	unsigned long offset;		/* nsec offset at last update of interpolator */
	u64 last_counter;		/* counter value in units of the counter at last update */
	u64 last_cycle;			/* Last timer value if TIME_SOURCE_JITTER is set */
	u64 frequency;			/* frequency in counts/second */
	long drift;			/* drift in parts-per-million (or -1) */
	unsigned long skips;		/* skips forward */
	unsigned long ns_skipped;	/* nanoseconds skipped */
	struct time_interpolator *next;
};

extern void register_time_interpolator(struct time_interpolator *);
extern void unregister_time_interpolator(struct time_interpolator *);
extern void time_interpolator_reset(void);
extern unsigned long time_interpolator_get_offset(void);

#else /* !CONFIG_TIME_INTERPOLATION */

static inline void
time_interpolator_reset(void)
{
}

#endif /* !CONFIG_TIME_INTERPOLATION */

#define TICK_LENGTH_SHIFT	32

/* Returns how long ticks are at present, in ns / 2^(SHIFT_SCALE-10). */
extern u64 current_tick_length(void);

extern int do_adjtimex(struct timex *);

#endif /* KERNEL */

#endif /* LINUX_TIMEX_H */
