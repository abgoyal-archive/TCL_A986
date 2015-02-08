



#ifndef __FCS_AUTH_H__
#define __FCS_AUTH_H__

#include <fcs/bfa_fcs.h>
#include <fcs/bfa_fcs_vport.h>
#include <fcs/bfa_fcs_lport.h>

void bfa_fcs_auth_uf_recv(struct bfa_fcs_fabric_s *fabric, int len);
void bfa_fcs_auth_start(struct bfa_fcs_fabric_s *fabric);
void bfa_fcs_auth_stop(struct bfa_fcs_fabric_s *fabric);

#endif /* __FCS_UF_H__ */
