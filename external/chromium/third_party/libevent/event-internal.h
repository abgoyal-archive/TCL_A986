#ifndef _EVENT_INTERNAL_H_
#define _EVENT_INTERNAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"
#include "min_heap.h"
#include "evsignal.h"

struct eventop {
	const char *name;
	void *(*init)(struct event_base *);
	int (*add)(void *, struct event *);
	int (*del)(void *, struct event *);
	int (*dispatch)(struct event_base *, void *, struct timeval *);
	void (*dealloc)(struct event_base *, void *);
	/* set if we need to reinitialize the event base */
	int need_reinit;
};

struct event_base {
	const struct eventop *evsel;
	void *evbase;
	int event_count;		/* counts number of total events */
	int event_count_active;	/* counts number of active events */

	int event_gotterm;		/* Set to terminate loop */
	int event_break;		/* Set to terminate loop immediately */

	/* active event management */
	struct event_list **activequeues;
	int nactivequeues;

	/* signal handling info */
	struct evsignal_info sig;

	struct event_list eventqueue;
	struct timeval event_tv;

	struct min_heap timeheap;

	struct timeval tv_cache;
};

/* Internal use only: Functions that might be missing from <sys/queue.h> */
#ifndef HAVE_TAILQFOREACH
#define	TAILQ_FIRST(head)		((head)->tqh_first)
#define	TAILQ_END(head)			NULL
#define	TAILQ_NEXT(elm, field)		((elm)->field.tqe_next)
#define TAILQ_FOREACH(var, head, field)					\
	for((var) = TAILQ_FIRST(head);					\
	    (var) != TAILQ_END(head);					\
	    (var) = TAILQ_NEXT(var, field))
#define	TAILQ_INSERT_BEFORE(listelm, elm, field) do {			\
	(elm)->field.tqe_prev = (listelm)->field.tqe_prev;		\
	(elm)->field.tqe_next = (listelm);				\
	*(listelm)->field.tqe_prev = (elm);				\
	(listelm)->field.tqe_prev = &(elm)->field.tqe_next;		\
} while (0)
#endif /* TAILQ_FOREACH */

int _evsignal_set_handler(struct event_base *base, int evsignal,
			  void (*fn)(int));
int _evsignal_restore_handler(struct event_base *base, int evsignal);

/* defined in evutil.c */
const char *evutil_getenv(const char *varname);

#ifdef __cplusplus
}
#endif

#endif /* _EVENT_INTERNAL_H_ */
