

#ifndef __BFA_LPS_PRIV_H__
#define __BFA_LPS_PRIV_H__

#include <bfa_svc.h>

struct bfa_lps_mod_s {
	struct list_head		lps_free_q;
	struct list_head		lps_active_q;
	struct bfa_lps_s	*lps_arr;
	int			num_lps;
};

#define BFA_LPS_MOD(__bfa)		(&(__bfa)->modules.lps_mod)
#define BFA_LPS_FROM_TAG(__mod, __tag)	(&(__mod)->lps_arr[__tag])

void	bfa_lps_isr(struct bfa_s *bfa, struct bfi_msg_s *msg);

#endif /* __BFA_LPS_PRIV_H__ */
