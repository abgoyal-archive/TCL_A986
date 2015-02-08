
#ifndef __FCS_FCPIM_H__
#define __FCS_FCPIM_H__

#include <defs/bfa_defs_port.h>
#include <fcs/bfa_fcs_lport.h>
#include <fcs/bfa_fcs_rport.h>

struct bfa_fcs_itnim_s *bfa_fcs_itnim_create(struct bfa_fcs_rport_s *rport);
void bfa_fcs_itnim_delete(struct bfa_fcs_itnim_s *itnim);
void bfa_fcs_itnim_rport_offline(struct bfa_fcs_itnim_s *itnim);
void bfa_fcs_itnim_rport_online(struct bfa_fcs_itnim_s *itnim);
bfa_status_t bfa_fcs_itnim_get_online_state(struct bfa_fcs_itnim_s *itnim);

void bfa_fcs_itnim_is_initiator(struct bfa_fcs_itnim_s *itnim);
void bfa_fcs_itnim_pause(struct bfa_fcs_itnim_s *itnim);
void bfa_fcs_itnim_resume(struct bfa_fcs_itnim_s *itnim);

void bfa_fcs_fcpim_uf_recv(struct bfa_fcs_itnim_s *itnim, struct fchs_s *fchs,
			u16 len);
#endif /* __FCS_FCPIM_H__ */
