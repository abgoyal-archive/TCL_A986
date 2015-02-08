


#include "fcs_fabric.h"
#include "fcs_trcmod.h"

BFA_TRC_FILE(FCS, VFAPI);


bfa_status_t
bfa_fcs_vf_mode_enable(struct bfa_fcs_s *fcs, u16 vf_id)
{
	return BFA_STATUS_OK;
}

bfa_status_t
bfa_fcs_vf_mode_disable(struct bfa_fcs_s *fcs)
{
	return BFA_STATUS_OK;
}

bfa_status_t
bfa_fcs_vf_create(bfa_fcs_vf_t *vf, struct bfa_fcs_s *fcs, u16 vf_id,
		  struct bfa_port_cfg_s *port_cfg, struct bfad_vf_s *vf_drv)
{
	bfa_trc(fcs, vf_id);
	return BFA_STATUS_OK;
}

bfa_status_t
bfa_fcs_vf_delete(bfa_fcs_vf_t *vf)
{
	bfa_trc(vf->fcs, vf->vf_id);
	return BFA_STATUS_OK;
}

void
bfa_fcs_vf_start(bfa_fcs_vf_t *vf)
{
	bfa_trc(vf->fcs, vf->vf_id);
}

bfa_status_t
bfa_fcs_vf_stop(bfa_fcs_vf_t *vf)
{
	bfa_trc(vf->fcs, vf->vf_id);
	return BFA_STATUS_OK;
}

void
bfa_fcs_vf_get_attr(bfa_fcs_vf_t *vf, struct bfa_vf_attr_s *vf_attr)
{
	bfa_trc(vf->fcs, vf->vf_id);
}

void
bfa_fcs_vf_get_stats(bfa_fcs_vf_t *vf, struct bfa_vf_stats_s *vf_stats)
{
	bfa_os_memcpy(vf_stats, &vf->stats, sizeof(struct bfa_vf_stats_s));
	return;
}

void
bfa_fcs_vf_clear_stats(bfa_fcs_vf_t *vf)
{
	bfa_os_memset(&vf->stats, 0, sizeof(struct bfa_vf_stats_s));
	return;
}

bfa_fcs_vf_t   *
bfa_fcs_vf_lookup(struct bfa_fcs_s *fcs, u16 vf_id)
{
	bfa_trc(fcs, vf_id);
	if (vf_id == FC_VF_ID_NULL)
		return &fcs->fabric;

	/**
	 * @todo vf support
	 */

	return NULL;
}

struct bfad_vf_s      *
bfa_fcs_vf_get_drv_vf(bfa_fcs_vf_t *vf)
{
	bfa_assert(vf);
	bfa_trc(vf->fcs, vf->vf_id);
	return vf->vf_drv;
}

void
bfa_fcs_vf_list(struct bfa_fcs_s *fcs, u16 *vf_ids, int *nvfs)
{
	bfa_trc(fcs, *nvfs);
}

void
bfa_fcs_vf_list_all(struct bfa_fcs_s *fcs, u16 *vf_ids, int *nvfs)
{
	bfa_trc(fcs, *nvfs);
}

void
bfa_fcs_vf_get_ports(bfa_fcs_vf_t *vf, wwn_t lpwwn[], int *nlports)
{
	struct list_head        *qe;
	struct bfa_fcs_vport_s *vport;
	int             i;
	struct bfa_fcs_s      *fcs;

	if (vf == NULL || lpwwn == NULL || *nlports == 0)
		return;

	fcs = vf->fcs;

	bfa_trc(fcs, vf->vf_id);
	bfa_trc(fcs, (u32) *nlports);

	i = 0;
	lpwwn[i++] = vf->bport.port_cfg.pwwn;

	list_for_each(qe, &vf->vport_q) {
		if (i >= *nlports)
			break;

		vport = (struct bfa_fcs_vport_s *) qe;
		lpwwn[i++] = vport->lport.port_cfg.pwwn;
	}

	bfa_trc(fcs, i);
	*nlports = i;
	return;
}


