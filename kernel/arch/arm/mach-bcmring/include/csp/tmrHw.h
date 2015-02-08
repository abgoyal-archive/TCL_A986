

/****************************************************************************/
/****************************************************************************/
#ifndef _TMRHW_H
#define _TMRHW_H

#include <csp/stdint.h>

typedef uint32_t tmrHw_ID_t;	/* Timer ID */
typedef uint32_t tmrHw_COUNT_t;	/* Timer count */
typedef uint32_t tmrHw_INTERVAL_t;	/* Timer interval */
typedef uint32_t tmrHw_RATE_t;	/* Timer event (count/interrupt) rate */

typedef enum {
	tmrHw_INTERRUPT_STATUS_SET,	/* Interrupted  */
	tmrHw_INTERRUPT_STATUS_UNSET	/* No Interrupt */
} tmrHw_INTERRUPT_STATUS_e;

typedef enum {
	tmrHw_CAPABILITY_CLOCK,	/* Clock speed in HHz */
	tmrHw_CAPABILITY_RESOLUTION	/* Timer resolution in bits */
} tmrHw_CAPABILITY_e;

/****************************************************************************/
/****************************************************************************/
uint32_t tmrHw_getTimerCapability(tmrHw_ID_t timerId,	/*  [ IN ] Timer Id */
				  tmrHw_CAPABILITY_e capability	/*  [ IN ] Timer capability */
);

/****************************************************************************/
/****************************************************************************/
tmrHw_RATE_t tmrHw_setPeriodicTimerRate(tmrHw_ID_t timerId,	/*  [ IN ] Timer Id */
					tmrHw_RATE_t rate	/*  [ IN ] Number of timer interrupt per second */
);

/****************************************************************************/
/****************************************************************************/
tmrHw_INTERVAL_t tmrHw_setPeriodicTimerInterval(tmrHw_ID_t timerId,	/*  [ IN ] Timer Id */
						tmrHw_INTERVAL_t msec	/*  [ IN ] Interval in mili-second */
);

/****************************************************************************/
/****************************************************************************/
tmrHw_INTERVAL_t tmrHw_setOneshotTimerInterval(tmrHw_ID_t timerId,	/*  [ IN ] Timer Id */
					       tmrHw_INTERVAL_t msec	/*  [ IN ] Interval in mili-second */
);

/****************************************************************************/
/****************************************************************************/
tmrHw_RATE_t tmrHw_setFreeRunningTimer(tmrHw_ID_t timerId,	/*  [ IN ] Timer Id */
				       uint32_t divider	/*  [ IN ] Dividing the clock frequency */
) __attribute__ ((section(".aramtext")));

/****************************************************************************/
/****************************************************************************/
int tmrHw_startTimer(tmrHw_ID_t timerId	/*  [ IN ] Timer id */
) __attribute__ ((section(".aramtext")));

/****************************************************************************/
/****************************************************************************/
int tmrHw_stopTimer(tmrHw_ID_t timerId	/*  [ IN ] Timer id */
);

/****************************************************************************/
/****************************************************************************/
tmrHw_COUNT_t tmrHw_GetCurrentCount(tmrHw_ID_t timerId	/*  [ IN ] Timer id */
) __attribute__ ((section(".aramtext")));

/****************************************************************************/
/****************************************************************************/
tmrHw_RATE_t tmrHw_getCountRate(tmrHw_ID_t timerId	/*  [ IN ] Timer id */
) __attribute__ ((section(".aramtext")));

/****************************************************************************/
/****************************************************************************/
void tmrHw_enableInterrupt(tmrHw_ID_t timerId	/*  [ IN ] Timer id */
);

/****************************************************************************/
/****************************************************************************/
void tmrHw_disableInterrupt(tmrHw_ID_t timerId	/*  [ IN ] Timer id */
);

/****************************************************************************/
/****************************************************************************/
void tmrHw_clearInterrupt(tmrHw_ID_t timerId	/*  [ IN ] Timer id */
);

/****************************************************************************/
/****************************************************************************/
tmrHw_INTERRUPT_STATUS_e tmrHw_getInterruptStatus(tmrHw_ID_t timerId	/*  [ IN ] Timer id */
);

/****************************************************************************/
/****************************************************************************/
tmrHw_ID_t tmrHw_getInterruptSource(void);

/****************************************************************************/
/****************************************************************************/
void tmrHw_printDebugInfo(tmrHw_ID_t timerId,	/*  [ IN ] Timer id */
			  int (*fpPrint) (const char *, ...)	/*  [ IN ] Print callback function */
);

/****************************************************************************/
/****************************************************************************/
void tmrHw_udelay(tmrHw_ID_t timerId,	/*  [ IN ] Timer id */
		  unsigned long usecs	/*  [ IN ] usec to delay */
) __attribute__ ((section(".aramtext")));

#endif /* _TMRHW_H */
