

#include <bfa_timer.h>
#include <cs/bfa_debug.h>

void
bfa_timer_init(struct bfa_timer_mod_s *mod)
{
	INIT_LIST_HEAD(&mod->timer_q);
}

void
bfa_timer_beat(struct bfa_timer_mod_s *mod)
{
	struct list_head        *qh = &mod->timer_q;
	struct list_head        *qe, *qe_next;
	struct bfa_timer_s *elem;
	struct list_head         timedout_q;

	INIT_LIST_HEAD(&timedout_q);

	qe = bfa_q_next(qh);

	while (qe != qh) {
		qe_next = bfa_q_next(qe);

		elem = (struct bfa_timer_s *) qe;
		if (elem->timeout <= BFA_TIMER_FREQ) {
			elem->timeout = 0;
			list_del(&elem->qe);
			list_add_tail(&elem->qe, &timedout_q);
		} else {
			elem->timeout -= BFA_TIMER_FREQ;
		}

		qe = qe_next;	/* go to next elem */
	}

	/*
	 * Pop all the timeout entries
	 */
	while (!list_empty(&timedout_q)) {
		bfa_q_deq(&timedout_q, &elem);
		elem->timercb(elem->arg);
	}
}

void
bfa_timer_begin(struct bfa_timer_mod_s *mod, struct bfa_timer_s *timer,
		    void (*timercb) (void *), void *arg, unsigned int timeout)
{

	bfa_assert(timercb != NULL);
	bfa_assert(!bfa_q_is_on_q(&mod->timer_q, timer));

	timer->timeout = timeout;
	timer->timercb = timercb;
	timer->arg = arg;

	list_add_tail(&timer->qe, &mod->timer_q);
}

void
bfa_timer_stop(struct bfa_timer_s *timer)
{
	bfa_assert(!list_empty(&timer->qe));

	list_del(&timer->qe);
}
