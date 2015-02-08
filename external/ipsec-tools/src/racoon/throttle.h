/*	$NetBSD: throttle.h,v 1.4 2006/09/09 16:22:10 manu Exp $	*/

/* Id: throttle.h,v 1.1 2004/11/30 00:46:09 manubsd Exp */


#ifndef _THROTTLE_H
#define _THROTTLE_H

struct throttle_entry {
	int penalty;
	TAILQ_ENTRY(throttle_entry) next;
	struct sockaddr_storage host;
};

TAILQ_HEAD(throttle_list, throttle_entry);

#define THROTTLE_PENALTY 1
#define THROTTLE_PENALTY_MAX 10

struct throttle_entry *throttle_add(struct sockaddr *);
int throttle_host(struct sockaddr *, int);

#endif /* _THROTTLE_H */
