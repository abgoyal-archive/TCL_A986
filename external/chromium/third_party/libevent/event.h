#ifndef _EVENT_H_
#define _EVENT_H_



#ifdef __cplusplus
extern "C" {
#endif

#include "event-config.h"
#ifdef _EVENT_HAVE_SYS_TYPES_H
#include <sys/types.h>
#endif
#ifdef _EVENT_HAVE_SYS_TIME_H
#include <sys/time.h>
#endif
#ifdef _EVENT_HAVE_STDINT_H
#include <stdint.h>
#endif
#include <stdarg.h>

/* For int types. */
#include "evutil.h"

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#undef WIN32_LEAN_AND_MEAN
typedef unsigned char u_char;
typedef unsigned short u_short;
#endif

#define EVLIST_TIMEOUT	0x01
#define EVLIST_INSERTED	0x02
#define EVLIST_SIGNAL	0x04
#define EVLIST_ACTIVE	0x08
#define EVLIST_INTERNAL	0x10
#define EVLIST_INIT	0x80

/* EVLIST_X_ Private space: 0x1000-0xf000 */
#define EVLIST_ALL	(0xf000 | 0x9f)

#define EV_TIMEOUT	0x01
#define EV_READ		0x02
#define EV_WRITE	0x04
#define EV_SIGNAL	0x08
#define EV_PERSIST	0x10	/* Persistant event */

/* Fix so that ppl dont have to run with <sys/queue.h> */
#ifndef TAILQ_ENTRY
#define _EVENT_DEFINED_TQENTRY
#define TAILQ_ENTRY(type)						\
struct {								\
	struct type *tqe_next;	/* next element */			\
	struct type **tqe_prev;	/* address of previous next element */	\
}
#endif /* !TAILQ_ENTRY */

struct event_base;
#ifndef EVENT_NO_STRUCT
struct event {
	TAILQ_ENTRY (event) ev_next;
	TAILQ_ENTRY (event) ev_active_next;
	TAILQ_ENTRY (event) ev_signal_next;
	unsigned int min_heap_idx;	/* for managing timeouts */

	struct event_base *ev_base;

	int ev_fd;
	short ev_events;
	short ev_ncalls;
	short *ev_pncalls;	/* Allows deletes in callback */

	struct timeval ev_timeout;

	int ev_pri;		/* smaller numbers are higher priority */

	void (*ev_callback)(int, short, void *arg);
	void *ev_arg;

	int ev_res;		/* result passed to event callback */
	int ev_flags;
};
#else
struct event;
#endif

#define EVENT_SIGNAL(ev)	(int)(ev)->ev_fd
#define EVENT_FD(ev)		(int)(ev)->ev_fd

struct evkeyval {
	TAILQ_ENTRY(evkeyval) next;

	char *key;
	char *value;
};

#ifdef _EVENT_DEFINED_TQENTRY
#undef TAILQ_ENTRY
struct event_list;
struct evkeyvalq;
#undef _EVENT_DEFINED_TQENTRY
#else
TAILQ_HEAD (event_list, event);
TAILQ_HEAD (evkeyvalq, evkeyval);
#endif /* _EVENT_DEFINED_TQENTRY */

struct event_base *event_base_new(void);

struct event_base *event_init(void);

int event_reinit(struct event_base *base);

int event_dispatch(void);


int event_base_dispatch(struct event_base *);


const char *event_base_get_method(struct event_base *);
        
        
void event_base_free(struct event_base *);


#define _EVENT_LOG_DEBUG 0
#define _EVENT_LOG_MSG   1
#define _EVENT_LOG_WARN  2
#define _EVENT_LOG_ERR   3
typedef void (*event_log_cb)(int severity, const char *msg);
void event_set_log_callback(event_log_cb cb);

int event_base_set(struct event_base *, struct event *);

/*@{*/
#define EVLOOP_ONCE	0x01	/**< Block at most once. */
#define EVLOOP_NONBLOCK	0x02	/**< Do not block. */
/*@}*/

int event_loop(int);

int event_base_loop(struct event_base *, int);

int event_loopexit(const struct timeval *);


int event_base_loopexit(struct event_base *, const struct timeval *);

int event_loopbreak(void);

int event_base_loopbreak(struct event_base *);


#define evtimer_add(ev, tv)		event_add(ev, tv)


#define evtimer_set(ev, cb, arg)	event_set(ev, -1, 0, cb, arg)


#define evtimer_del(ev)			event_del(ev)
#define evtimer_pending(ev, tv)		event_pending(ev, EV_TIMEOUT, tv)
#define evtimer_initialized(ev)		((ev)->ev_flags & EVLIST_INIT)

#define timeout_add(ev, tv)		event_add(ev, tv)


#define timeout_set(ev, cb, arg)	event_set(ev, -1, 0, cb, arg)


#define timeout_del(ev)			event_del(ev)

#define timeout_pending(ev, tv)		event_pending(ev, EV_TIMEOUT, tv)
#define timeout_initialized(ev)		((ev)->ev_flags & EVLIST_INIT)

#define signal_add(ev, tv)		event_add(ev, tv)
#define signal_set(ev, x, cb, arg)	\
	event_set(ev, x, EV_SIGNAL|EV_PERSIST, cb, arg)
#define signal_del(ev)			event_del(ev)
#define signal_pending(ev, tv)		event_pending(ev, EV_SIGNAL, tv)
#define signal_initialized(ev)		((ev)->ev_flags & EVLIST_INIT)

void event_set(struct event *, int, short, void (*)(int, short, void *), void *);

int event_once(int, short, void (*)(int, short, void *), void *,
    const struct timeval *);


int event_base_once(struct event_base *base, int fd, short events,
    void (*callback)(int, short, void *), void *arg,
    const struct timeval *timeout);


int event_add(struct event *ev, const struct timeval *timeout);


int event_del(struct event *);

void event_active(struct event *, int, short);


int event_pending(struct event *ev, short event, struct timeval *tv);


#ifdef WIN32
#define event_initialized(ev)		((ev)->ev_flags & EVLIST_INIT && (ev)->ev_fd != (int)INVALID_HANDLE_VALUE)
#else
#define event_initialized(ev)		((ev)->ev_flags & EVLIST_INIT)
#endif


const char *event_get_version(void);


const char *event_get_method(void);


int	event_priority_init(int);


int	event_base_priority_init(struct event_base *, int);


int	event_priority_set(struct event *, int);


/* These functions deal with buffering input and output */

struct evbuffer {
	u_char *buffer;
	u_char *orig_buffer;

	size_t misalign;
	size_t totallen;
	size_t off;

	void (*cb)(struct evbuffer *, size_t, size_t, void *);
	void *cbarg;
};

/* Just for error reporting - use other constants otherwise */
#define EVBUFFER_READ		0x01
#define EVBUFFER_WRITE		0x02
#define EVBUFFER_EOF		0x10
#define EVBUFFER_ERROR		0x20
#define EVBUFFER_TIMEOUT	0x40

struct bufferevent;
typedef void (*evbuffercb)(struct bufferevent *, void *);
typedef void (*everrorcb)(struct bufferevent *, short what, void *);

struct event_watermark {
	size_t low;
	size_t high;
};

#ifndef EVENT_NO_STRUCT
struct bufferevent {
	struct event_base *ev_base;

	struct event ev_read;
	struct event ev_write;

	struct evbuffer *input;
	struct evbuffer *output;

	struct event_watermark wm_read;
	struct event_watermark wm_write;

	evbuffercb readcb;
	evbuffercb writecb;
	everrorcb errorcb;
	void *cbarg;

	int timeout_read;	/* in seconds */
	int timeout_write;	/* in seconds */

	short enabled;	/* events that are currently enabled */
};
#endif

struct bufferevent *bufferevent_new(int fd,
    evbuffercb readcb, evbuffercb writecb, everrorcb errorcb, void *cbarg);


int bufferevent_base_set(struct event_base *base, struct bufferevent *bufev);


int bufferevent_priority_set(struct bufferevent *bufev, int pri);


void bufferevent_free(struct bufferevent *bufev);


void bufferevent_setcb(struct bufferevent *bufev,
    evbuffercb readcb, evbuffercb writecb, everrorcb errorcb, void *cbarg);

void bufferevent_setfd(struct bufferevent *bufev, int fd);

int bufferevent_write(struct bufferevent *bufev,
    const void *data, size_t size);


int bufferevent_write_buffer(struct bufferevent *bufev, struct evbuffer *buf);


size_t bufferevent_read(struct bufferevent *bufev, void *data, size_t size);

int bufferevent_enable(struct bufferevent *bufev, short event);


int bufferevent_disable(struct bufferevent *bufev, short event);


void bufferevent_settimeout(struct bufferevent *bufev,
    int timeout_read, int timeout_write);



void bufferevent_setwatermark(struct bufferevent *bufev, short events,
    size_t lowmark, size_t highmark);

#define EVBUFFER_LENGTH(x)	(x)->off
#define EVBUFFER_DATA(x)	(x)->buffer
#define EVBUFFER_INPUT(x)	(x)->input
#define EVBUFFER_OUTPUT(x)	(x)->output


struct evbuffer *evbuffer_new(void);


void evbuffer_free(struct evbuffer *);


int evbuffer_expand(struct evbuffer *, size_t);


int evbuffer_add(struct evbuffer *, const void *, size_t);



int evbuffer_remove(struct evbuffer *, void *, size_t);


char *evbuffer_readline(struct evbuffer *);


int evbuffer_add_buffer(struct evbuffer *, struct evbuffer *);


int evbuffer_add_printf(struct evbuffer *, const char *fmt, ...)
#ifdef __GNUC__
  __attribute__((format(printf, 2, 3)))
#endif
;


int evbuffer_add_vprintf(struct evbuffer *, const char *fmt, va_list ap);


void evbuffer_drain(struct evbuffer *, size_t);


int evbuffer_write(struct evbuffer *, int);


int evbuffer_read(struct evbuffer *, int, int);


u_char *evbuffer_find(struct evbuffer *, const u_char *, size_t);

void evbuffer_setcb(struct evbuffer *, void (*)(struct evbuffer *, size_t, size_t, void *), void *);


void evtag_init(void);

void evtag_marshal(struct evbuffer *evbuf, ev_uint32_t tag, const void *data,
    ev_uint32_t len);

void encode_int(struct evbuffer *evbuf, ev_uint32_t number);

void evtag_marshal_int(struct evbuffer *evbuf, ev_uint32_t tag,
    ev_uint32_t integer);

void evtag_marshal_string(struct evbuffer *buf, ev_uint32_t tag,
    const char *string);

void evtag_marshal_timeval(struct evbuffer *evbuf, ev_uint32_t tag,
    struct timeval *tv);

int evtag_unmarshal(struct evbuffer *src, ev_uint32_t *ptag,
    struct evbuffer *dst);
int evtag_peek(struct evbuffer *evbuf, ev_uint32_t *ptag);
int evtag_peek_length(struct evbuffer *evbuf, ev_uint32_t *plength);
int evtag_payload_length(struct evbuffer *evbuf, ev_uint32_t *plength);
int evtag_consume(struct evbuffer *evbuf);

int evtag_unmarshal_int(struct evbuffer *evbuf, ev_uint32_t need_tag,
    ev_uint32_t *pinteger);

int evtag_unmarshal_fixed(struct evbuffer *src, ev_uint32_t need_tag,
    void *data, size_t len);

int evtag_unmarshal_string(struct evbuffer *evbuf, ev_uint32_t need_tag,
    char **pstring);

int evtag_unmarshal_timeval(struct evbuffer *evbuf, ev_uint32_t need_tag,
    struct timeval *ptv);

#ifdef __cplusplus
}
#endif

#endif /* _EVENT_H_ */
