


#ifndef __BFA_Q_H__
#define __BFA_Q_H__

#define bfa_q_first(_q) ((void *)(((struct list_head *) (_q))->next))
#define bfa_q_next(_qe)	(((struct list_head *) (_qe))->next)
#define bfa_q_prev(_qe) (((struct list_head *) (_qe))->prev)

#define bfa_q_qe_init(_qe) {						\
	bfa_q_next(_qe) = (struct list_head *) NULL;			\
	bfa_q_prev(_qe) = (struct list_head *) NULL;			\
}

#define bfa_q_deq(_q, _qe) {						\
	if (!list_empty(_q)) {					\
		(*((struct list_head **) (_qe))) = bfa_q_next(_q);	\
		bfa_q_prev(bfa_q_next(*((struct list_head **) _qe))) =	\
						(struct list_head *) (_q); \
		bfa_q_next(_q) = bfa_q_next(*((struct list_head **) _qe)); \
		BFA_Q_DBG_INIT(*((struct list_head **) _qe));		\
	} else {							\
		*((struct list_head **) (_qe)) = (struct list_head *) NULL; \
	}								\
}

#define bfa_q_deq_tail(_q, _qe) {					    \
	if (!list_empty(_q)) {					            \
		*((struct list_head **) (_qe)) = bfa_q_prev(_q);	    \
		bfa_q_next(bfa_q_prev(*((struct list_head **) _qe))) = 	    \
						(struct list_head *) (_q);  \
		bfa_q_prev(_q) = bfa_q_prev(*(struct list_head **) _qe);    \
		BFA_Q_DBG_INIT(*((struct list_head **) _qe));		    \
	} else {							    \
		*((struct list_head **) (_qe)) = (struct list_head *) NULL; \
	}								    \
}

#ifndef BFA_PERF_BUILD
#define BFA_Q_DBG_INIT(_qe)	bfa_q_qe_init(_qe)
#else
#define BFA_Q_DBG_INIT(_qe)
#endif

#define bfa_q_is_on_q(_q, _qe)		\
	bfa_q_is_on_q_func(_q, (struct list_head *)(_qe))
extern int bfa_q_is_on_q_func(struct list_head *q, struct list_head *qe);

#endif
