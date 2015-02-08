

#include <bfa.h>
#include <bfa_svc.h>
#include "fcs_lport.h"
#include "fcs_rport.h"
#include "lport_priv.h"



void
bfa_fcs_port_fab_init(struct bfa_fcs_port_s *port)
{
	bfa_fcs_port_ns_init(port);
	bfa_fcs_port_scn_init(port);
	bfa_fcs_port_ms_init(port);
}

void
bfa_fcs_port_fab_online(struct bfa_fcs_port_s *port)
{
	bfa_fcs_port_ns_online(port);
	bfa_fcs_port_scn_online(port);
}

void
bfa_fcs_port_fab_offline(struct bfa_fcs_port_s *port)
{
	bfa_fcs_port_ns_offline(port);
	bfa_fcs_port_scn_offline(port);
	bfa_fcs_port_ms_offline(port);
}
