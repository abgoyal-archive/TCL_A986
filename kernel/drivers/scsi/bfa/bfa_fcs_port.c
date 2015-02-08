


#include <fcs/bfa_fcs.h>
#include <bfa_svc.h>
#include <fcs/bfa_fcs_fabric.h>
#include "fcs_trcmod.h"
#include "fcs.h"
#include "fcs_fabric.h"
#include "fcs_port.h"

BFA_TRC_FILE(FCS, PPORT);

static void
bfa_fcs_pport_event_handler(void *cbarg, bfa_pport_event_t event)
{
	struct bfa_fcs_s      *fcs = cbarg;

	bfa_trc(fcs, event);

	switch (event) {
	case BFA_PPORT_LINKUP:
		bfa_fcs_fabric_link_up(&fcs->fabric);
		break;

	case BFA_PPORT_LINKDOWN:
		bfa_fcs_fabric_link_down(&fcs->fabric);
		break;

	case BFA_PPORT_TRUNK_LINKDOWN:
		bfa_assert(0);
		break;

	default:
		bfa_assert(0);
	}
}

void
bfa_fcs_pport_attach(struct bfa_fcs_s *fcs)
{
	bfa_fcport_event_register(fcs->bfa, bfa_fcs_pport_event_handler, fcs);
}
