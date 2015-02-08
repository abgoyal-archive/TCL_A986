


#include "bfa_os_inc.h"
#include "fcs_rport.h"
#include "fcs_fcptm.h"
#include "fcs/bfa_fcs_rport.h"

struct bfa_fcs_tin_s *
bfa_fcs_tin_create(struct bfa_fcs_rport_s *rport)
{
	return NULL;
}

void
bfa_fcs_tin_delete(struct bfa_fcs_tin_s *tin)
{
}

void
bfa_fcs_tin_rport_offline(struct bfa_fcs_tin_s *tin)
{
}

void
bfa_fcs_tin_rport_online(struct bfa_fcs_tin_s *tin)
{
}

void
bfa_fcs_tin_rx_prli(struct bfa_fcs_tin_s *tin, struct fchs_s *fchs, u16 len)
{
}

void
bfa_fcs_fcptm_uf_recv(struct bfa_fcs_tin_s *tin, struct fchs_s *fchs, u16 len)
{
}

void
bfa_fcs_tin_pause(struct bfa_fcs_tin_s *tin)
{
}

void
bfa_fcs_tin_resume(struct bfa_fcs_tin_s *tin)
{
}
