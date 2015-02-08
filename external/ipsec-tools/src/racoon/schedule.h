/*	$NetBSD: schedule.h,v 1.4.6.1 2007/03/21 14:29:48 vanhu Exp $	*/

/* Id: schedule.h,v 1.5 2006/05/03 21:53:42 vanhu Exp */


#ifndef _SCHEDULE_H
#define _SCHEDULE_H

#include <sys/queue.h>
#include "gnuc.h"

/* scheduling table */
/* the head is the nearest event. */
struct sched {
	time_t xtime;		/* event time which is as time(3). */
				/*
				 * if defined FIXY2038PROBLEM, this time
				 * is from the time when called sched_init().
				 */
	void (*func) __P((void *)); /* call this function when timeout. */
	void *param;		/* pointer to parameter */

	int dead;		/* dead or alive */
	long id;		/* for debug */
	time_t created;		/* for debug */
	time_t tick;		/* for debug */

	TAILQ_ENTRY(sched) chain;
};

/* cancel schedule */
#define SCHED_KILL(s)                                                          \
do {                                                                           \
	if(s != NULL){	   														\
		sched_kill(s);                                                         \
		s = NULL;                                                              \
	}\
} while(0)

/* must be called after it's called from scheduler. */
#define SCHED_INIT(s)	(s) = NULL

struct scheddump {
	time_t xtime;
	long id;
	time_t created;
	time_t tick;
};

struct timeval *schedular __P((void));
struct sched *sched_new __P((time_t, void (*func) __P((void *)), void *));
void sched_kill __P((struct sched *));
int sched_dump __P((caddr_t *, int *));
void sched_init __P((void));
void sched_scrub_param __P((void *));

#endif /* _SCHEDULE_H */
