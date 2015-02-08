
#ifndef __BFA_TIMER_H__
#define __BFA_TIMER_H__

#include <bfa_os_inc.h>
#include <cs/bfa_q.h>

struct bfa_s;

typedef void (*bfa_timer_cbfn_t)(void *);

struct bfa_timer_s {
	struct list_head	qe;
	bfa_timer_cbfn_t timercb;
	void            *arg;
	int             timeout;	/**< in millisecs. */
};

struct bfa_timer_mod_s {
	struct list_head timer_q;
};

#define BFA_TIMER_FREQ 200 /**< specified in millisecs */

void bfa_timer_beat(struct bfa_timer_mod_s *mod);
void bfa_timer_init(struct bfa_timer_mod_s *mod);
void bfa_timer_begin(struct bfa_timer_mod_s *mod, struct bfa_timer_s *timer,
			bfa_timer_cbfn_t timercb, void *arg,
			unsigned int timeout);
void bfa_timer_stop(struct bfa_timer_s *timer);

#endif /* __BFA_TIMER_H__ */
