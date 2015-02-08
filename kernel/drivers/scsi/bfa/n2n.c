

#include <bfa.h>
#include <bfa_svc.h>
#include "fcs_lport.h"
#include "fcs_rport.h"
#include "fcs_trcmod.h"
#include "lport_priv.h"

BFA_TRC_FILE(FCS, N2N);

void
bfa_fcs_port_n2n_init(struct bfa_fcs_port_s *port)
{
}

void
bfa_fcs_port_n2n_online(struct bfa_fcs_port_s *port)
{
	struct bfa_fcs_port_n2n_s *n2n_port = &port->port_topo.pn2n;
	struct bfa_port_cfg_s *pcfg = &port->port_cfg;
	struct bfa_fcs_rport_s *rport;

	bfa_trc(port->fcs, pcfg->pwwn);

	/*
	 * If our PWWN is > than that of the r-port, we have to initiate PLOGI
	 * and assign an Address. if not, we need to wait for its PLOGI.
	 *
	 * If our PWWN is < than that of the remote port, it will send a PLOGI
	 * with the PIDs assigned. The rport state machine take care of this
	 * incoming PLOGI.
	 */
	if (memcmp
	    ((void *)&pcfg->pwwn, (void *)&n2n_port->rem_port_wwn,
	     sizeof(wwn_t)) > 0) {
		port->pid = N2N_LOCAL_PID;
		/**
		 * First, check if we know the device by pwwn.
		 */
		rport = bfa_fcs_port_get_rport_by_pwwn(port,
						       n2n_port->rem_port_wwn);
		if (rport) {
			bfa_trc(port->fcs, rport->pid);
			bfa_trc(port->fcs, rport->pwwn);
			rport->pid = N2N_REMOTE_PID;
			bfa_fcs_rport_online(rport);
			return;
		}

		/*
		 * In n2n there can be only one rport. Delete the old one whose
		 * pid should be zero, because it is offline.
		 */
		if (port->num_rports > 0) {
			rport = bfa_fcs_port_get_rport_by_pid(port, 0);
			bfa_assert(rport != NULL);
			if (rport) {
				bfa_trc(port->fcs, rport->pwwn);
				bfa_fcs_rport_delete(rport);
			}
		}
		bfa_fcs_rport_create(port, N2N_REMOTE_PID);
	}
}

void
bfa_fcs_port_n2n_offline(struct bfa_fcs_port_s *port)
{
	struct bfa_fcs_port_n2n_s *n2n_port = &port->port_topo.pn2n;

	bfa_trc(port->fcs, port->pid);
	port->pid = 0;
	n2n_port->rem_port_wwn = 0;
	n2n_port->reply_oxid = 0;
}


