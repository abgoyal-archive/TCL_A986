

#ifndef _TCP_TIMER_H_
#define _TCP_TIMER_H_

#define	TCPT_NTIMERS	4

#define	TCPT_REXMT	0		/* retransmit */
#define	TCPT_PERSIST	1		/* retransmit persistence */
#define	TCPT_KEEP	2		/* keep alive */
#define	TCPT_2MSL	3		/* 2*msl quiet time timer */


#define TCPTV_MSL       ( 5*PR_SLOWHZ)          /* max seg lifetime (hah!) */

#define	TCPTV_SRTTBASE	0			/* base roundtrip time;
						   if 0, no idea yet */
#define	TCPTV_SRTTDFLT	(  3*PR_SLOWHZ)		/* assumed RTT if no info */

#define	TCPTV_PERSMIN	(  5*PR_SLOWHZ)		/* retransmit persistence */
#define	TCPTV_PERSMAX	( 60*PR_SLOWHZ)		/* maximum persist interval */

#define	TCPTV_KEEP_INIT	( 75*PR_SLOWHZ)		/* initial connect keep alive */
#define	TCPTV_KEEP_IDLE	(120*60*PR_SLOWHZ)	/* dflt time before probing */
#define	TCPTV_KEEPINTVL	( 75*PR_SLOWHZ)		/* default probe interval */
#define	TCPTV_KEEPCNT	8			/* max probes before drop */

#define	TCPTV_MIN	(  1*PR_SLOWHZ)		/* minimum allowable value */
/* #define	TCPTV_REXMTMAX	( 64*PR_SLOWHZ)	*/	/* max allowable REXMT value */
#define TCPTV_REXMTMAX  ( 12*PR_SLOWHZ)		/* max allowable REXMT value */

#define	TCP_LINGERTIME	120			/* linger at most 2 minutes */

#define TCP_MAXRXTSHIFT 12                      /* maximum retransmits */


#ifdef	TCPTIMERS
char *tcptimers[] =
    { "REXMT", "PERSIST", "KEEP", "2MSL" };
#endif

#define	TCPT_RANGESET(tv, value, tvmin, tvmax) { \
	(tv) = (value); \
	if ((tv) < (tvmin)) \
		(tv) = (tvmin); \
	else if ((tv) > (tvmax)) \
		(tv) = (tvmax); \
}

extern const int tcp_backoff[];

struct tcpcb;

void tcp_fasttimo _P((void));
void tcp_slowtimo _P((void));
void tcp_canceltimers _P((struct tcpcb *));

#endif
