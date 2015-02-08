

#ifndef __BFA_RPORT_PRIV_H__
#define __BFA_RPORT_PRIV_H__

#include <bfa_svc.h>

#define BFA_RPORT_MIN	4

struct bfa_rport_mod_s {
	struct bfa_rport_s *rps_list;	/*  list of rports	*/
	struct list_head 	rp_free_q;	/*  free bfa_rports	*/
	struct list_head 	rp_active_q;	/*  free bfa_rports 	*/
	u16	num_rports;	/*  number of rports	*/
};

#define BFA_RPORT_MOD(__bfa)	(&(__bfa)->modules.rport_mod)

#define BFA_RPORT_FROM_TAG(__bfa, _tag)				\
	(BFA_RPORT_MOD(__bfa)->rps_list +				\
	 ((_tag) & (BFA_RPORT_MOD(__bfa)->num_rports - 1)))

void	bfa_rport_isr(struct bfa_s *bfa, struct bfi_msg_s *msg);
#endif /* __BFA_RPORT_PRIV_H__ */
