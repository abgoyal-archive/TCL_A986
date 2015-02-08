

#ifndef __FCS_FCPTM_H__
#define __FCS_FCPTM_H__

#include <defs/bfa_defs_port.h>
#include <fcs/bfa_fcs_lport.h>
#include <fcs/bfa_fcs_rport.h>

struct bfa_fcs_tin_s *bfa_fcs_tin_create(struct bfa_fcs_rport_s *rport);
void bfa_fcs_tin_rport_offline(struct bfa_fcs_tin_s *tin);
void bfa_fcs_tin_rport_online(struct bfa_fcs_tin_s *tin);
void bfa_fcs_tin_delete(struct bfa_fcs_tin_s *tin);
void bfa_fcs_tin_rx_prli(struct bfa_fcs_tin_s *tin, struct fchs_s *fchs,
			u16 len);
void bfa_fcs_tin_pause(struct bfa_fcs_tin_s *tin);
void bfa_fcs_tin_resume(struct bfa_fcs_tin_s *tin);

void bfa_fcs_fcptm_modinit(struct bfa_fcs_s *fcs);
void bfa_fcs_fcptm_modexit(struct bfa_fcs_s *fcs);
void bfa_fcs_fcptm_uf_recv(struct bfa_fcs_tin_s *tin, struct fchs_s *fchs,
			u16 len);

#endif /* __FCS_FCPTM_H__ */
