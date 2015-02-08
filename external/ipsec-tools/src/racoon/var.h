/*	$NetBSD: var.h,v 1.4.6.1 2007/06/06 15:36:38 vanhu Exp $	*/

/* Id: var.h,v 1.6 2004/11/20 16:16:59 monas Exp */


#ifndef _VAR_H
#define _VAR_H

#if !defined(_VAR_H_)
#define _VAR_H_

#define MAX3(a, b, c) (a > b ? (a > c ? a : c) : (b > c ? b : c))

#define ISSET(exp, bit) (((exp) & (bit)) == (bit))

#define LALIGN(a) \
    ((a) > 0 ? ((a) &~ (sizeof(long) - 1)) : sizeof(long))

#define RNDUP(a) \
    ((a) > 0 ? (1 + (((a) - 1) | (sizeof(long) - 1))) : sizeof(long))

#define ARRAYLEN(a)	(sizeof(a)/sizeof(a[0]))

#define BUFSIZE    5120

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

#ifdef ENABLE_STATS
#include <sys/time.h>
#endif
#include <sys/socket.h>

#include <sys/socket.h>
#include <netdb.h>

/* var.h is used from non-racoon code (like eaytest), so we can't use niflags */
#define NIFLAGS	(NI_NUMERICHOST | NI_NUMERICSERV)

#define GETNAMEINFO(x, y, z) \
do { \
	if (getnameinfo((x), sysdep_sa_len(x), (y), sizeof(y), (z), sizeof(z), \
			NIFLAGS) != 0) { \
		if (y != NULL) \
			strncpy((y), "(invalid)", sizeof(y)); \
		if (z != NULL) \
			strncpy((z), "(invalid)", sizeof(z)); \
	} \
} while (0);

#define GETNAMEINFO_NULL(x, y) \
do { \
	if (getnameinfo((x), sysdep_sa_len(x), (y), sizeof(y), NULL, 0, \
			NIFLAGS) != 0) { \
		if (y != NULL) \
			strncpy((y), "(invalid)", sizeof(y)); \
	} \
} while (0);

#include <sys/queue.h>
#ifndef LIST_FOREACH
#define LIST_FOREACH(elm, head, field) \
	for (elm = LIST_FIRST(head); elm; elm = LIST_NEXT(elm, field))
#endif

#include "gcmalloc.h"

#endif /*!defined(_VAR_H_)*/

#endif /* _VAR_H */
