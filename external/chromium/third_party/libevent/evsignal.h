#ifndef _EVSIGNAL_H_
#define _EVSIGNAL_H_

typedef void (*ev_sighandler_t)(int);

struct evsignal_info {
	struct event ev_signal;
	int ev_signal_pair[2];
	int ev_signal_added;
	volatile sig_atomic_t evsignal_caught;
	struct event_list evsigevents[NSIG];
	sig_atomic_t evsigcaught[NSIG];
#ifdef HAVE_SIGACTION
	struct sigaction **sh_old;
#else
	ev_sighandler_t **sh_old;
#endif
	int sh_old_max;
};
int evsignal_init(struct event_base *);
void evsignal_process(struct event_base *);
int evsignal_add(struct event *);
int evsignal_del(struct event *);
void evsignal_dealloc(struct event_base *);

#endif /* _EVSIGNAL_H_ */
