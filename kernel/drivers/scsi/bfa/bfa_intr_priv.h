

#ifndef __BFA_INTR_PRIV_H__
#define __BFA_INTR_PRIV_H__

typedef void (*bfa_isr_func_t) (struct bfa_s *bfa, struct bfi_msg_s *m);
void bfa_isr_unhandled(struct bfa_s *bfa, struct bfi_msg_s *m);
void bfa_isr_bind(enum bfi_mclass mc, bfa_isr_func_t isr_func);


#define bfa_reqq_pi(__bfa, __reqq)	((__bfa)->iocfc.req_cq_pi[__reqq])
#define bfa_reqq_ci(__bfa, __reqq)					\
	(*(u32 *)((__bfa)->iocfc.req_cq_shadow_ci[__reqq].kva))

#define bfa_reqq_full(__bfa, __reqq)				\
	(((bfa_reqq_pi(__bfa, __reqq) + 1) &			\
	  ((__bfa)->iocfc.cfg.drvcfg.num_reqq_elems - 1)) ==	\
	 bfa_reqq_ci(__bfa, __reqq))

#define bfa_reqq_next(__bfa, __reqq)				\
	(bfa_reqq_full(__bfa, __reqq) ? NULL :			\
	 ((void *)((struct bfi_msg_s *)((__bfa)->iocfc.req_cq_ba[__reqq].kva) \
			  + bfa_reqq_pi((__bfa), (__reqq)))))

#define bfa_reqq_produce(__bfa, __reqq)	do {				\
	(__bfa)->iocfc.req_cq_pi[__reqq]++;				\
	(__bfa)->iocfc.req_cq_pi[__reqq] &=				\
		((__bfa)->iocfc.cfg.drvcfg.num_reqq_elems - 1);      \
	bfa_reg_write((__bfa)->iocfc.bfa_regs.cpe_q_pi[__reqq],		\
				(__bfa)->iocfc.req_cq_pi[__reqq]);      \
	bfa_os_mmiowb();      \
} while (0)

#define bfa_rspq_pi(__bfa, __rspq)					\
	(*(u32 *)((__bfa)->iocfc.rsp_cq_shadow_pi[__rspq].kva))

#define bfa_rspq_ci(__bfa, __rspq)	((__bfa)->iocfc.rsp_cq_ci[__rspq])
#define bfa_rspq_elem(__bfa, __rspq, __ci)				\
	(&((struct bfi_msg_s *)((__bfa)->iocfc.rsp_cq_ba[__rspq].kva))[__ci])

#define CQ_INCR(__index, __size) do {					\
			(__index)++;					\
			(__index) &= ((__size) - 1);      \
} while (0)

struct bfa_reqq_wait_s {
	struct list_head 	qe;
	void		(*qresume) (void *cbarg);
	void		*cbarg;
};

enum {
	BFA_REQQ_IOC	= 0,	/*  all low-priority IOC msgs	*/
	BFA_REQQ_FCXP	= 0,	/*  all FCXP messages		*/
	BFA_REQQ_LPS	= 0,	/*  all lport service msgs	*/
	BFA_REQQ_PORT	= 0,	/*  all port messages		*/
	BFA_REQQ_FLASH	= 0,	/*  for flash module		*/
	BFA_REQQ_DIAG	= 0,	/*  for diag module		*/
	BFA_REQQ_RPORT	= 0,	/*  all port messages		*/
	BFA_REQQ_SBOOT	= 0,	/*  all san boot messages	*/
	BFA_REQQ_QOS_LO	= 1,	/*  all low priority IO	*/
	BFA_REQQ_QOS_MD	= 2,	/*  all medium priority IO	*/
	BFA_REQQ_QOS_HI	= 3,	/*  all high priority IO	*/
};

static inline void
bfa_reqq_winit(struct bfa_reqq_wait_s *wqe, void (*qresume) (void *cbarg),
			void *cbarg)
{
	wqe->qresume = qresume;
	wqe->cbarg = cbarg;
}

#define bfa_reqq(__bfa, __reqq)	(&(__bfa)->reqq_waitq[__reqq])

#define bfa_reqq_wait(__bfa, __reqq, __wqe) do {			\
									\
		struct list_head *waitq = bfa_reqq(__bfa, __reqq);      \
									\
		bfa_assert(((__reqq) < BFI_IOC_MAX_CQS));      \
		bfa_assert((__wqe)->qresume && (__wqe)->cbarg);      \
									\
		list_add_tail(&(__wqe)->qe, waitq);      \
} while (0)

#define bfa_reqq_wcancel(__wqe)	list_del(&(__wqe)->qe)

#endif /* __BFA_INTR_PRIV_H__ */
