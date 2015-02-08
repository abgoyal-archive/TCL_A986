


#include <fcs/bfa_fcs.h>
#include "fcs_port.h"
#include "fcs_uf.h"
#include "fcs_vport.h"
#include "fcs_rport.h"
#include "fcs_fabric.h"
#include "fcs_fcpim.h"
#include "fcs_fcptm.h"
#include "fcbuild.h"
#include "fcs.h"
#include "bfad_drv.h"
#include <fcb/bfa_fcb.h>

struct bfa_fcs_mod_s {
	void		(*attach) (struct bfa_fcs_s *fcs);
	void            (*modinit) (struct bfa_fcs_s *fcs);
	void            (*modexit) (struct bfa_fcs_s *fcs);
};

#define BFA_FCS_MODULE(_mod) { _mod ## _modinit, _mod ## _modexit }

static struct bfa_fcs_mod_s fcs_modules[] = {
	{ bfa_fcs_pport_attach, NULL, NULL },
	{ bfa_fcs_uf_attach, NULL, NULL },
	{ bfa_fcs_fabric_attach, bfa_fcs_fabric_modinit,
	 bfa_fcs_fabric_modexit },
};


static void
bfa_fcs_exit_comp(void *fcs_cbarg)
{
	struct bfa_fcs_s *fcs = fcs_cbarg;
	struct bfad_s *bfad = fcs->bfad;

	complete(&bfad->comp);
}




void
bfa_fcs_attach(struct bfa_fcs_s *fcs, struct bfa_s *bfa, struct bfad_s *bfad,
			bfa_boolean_t min_cfg)
{
	int             i;
	struct bfa_fcs_mod_s  *mod;

	fcs->bfa = bfa;
	fcs->bfad = bfad;
	fcs->min_cfg = min_cfg;

	bfa_attach_fcs(bfa);
	fcbuild_init();

	for (i = 0; i < sizeof(fcs_modules) / sizeof(fcs_modules[0]); i++) {
		mod = &fcs_modules[i];
		if (mod->attach)
			mod->attach(fcs);
	}
}

void
bfa_fcs_init(struct bfa_fcs_s *fcs)
{
	int             i;
	struct bfa_fcs_mod_s  *mod;

	for (i = 0; i < sizeof(fcs_modules) / sizeof(fcs_modules[0]); i++) {
		mod = &fcs_modules[i];
		if (mod->modinit)
			mod->modinit(fcs);
	}
}

void
bfa_fcs_start(struct bfa_fcs_s *fcs)
{
	bfa_fcs_fabric_modstart(fcs);
}

void
bfa_fcs_driver_info_init(struct bfa_fcs_s *fcs,
			struct bfa_fcs_driver_info_s *driver_info)
{

	fcs->driver_info = *driver_info;

	bfa_fcs_fabric_psymb_init(&fcs->fabric);
}

void
bfa_fcs_set_fdmi_param(struct bfa_fcs_s *fcs, bfa_boolean_t fdmi_enable)
{

	fcs->fdmi_enabled = fdmi_enable;

}

void
bfa_fcs_exit(struct bfa_fcs_s *fcs)
{
	struct bfa_fcs_mod_s  *mod;
	int             nmods, i;

	bfa_wc_init(&fcs->wc, bfa_fcs_exit_comp, fcs);

	nmods = sizeof(fcs_modules) / sizeof(fcs_modules[0]);

	for (i = 0; i < nmods; i++) {

		mod = &fcs_modules[i];
		if (mod->modexit) {
			bfa_wc_up(&fcs->wc);
			mod->modexit(fcs);
		}
	}

	bfa_wc_wait(&fcs->wc);
}


void
bfa_fcs_trc_init(struct bfa_fcs_s *fcs, struct bfa_trc_mod_s *trcmod)
{
	fcs->trcmod = trcmod;
}


void
bfa_fcs_log_init(struct bfa_fcs_s *fcs, struct bfa_log_mod_s *logmod)
{
	fcs->logm = logmod;
}


void
bfa_fcs_aen_init(struct bfa_fcs_s *fcs, struct bfa_aen_s *aen)
{
	fcs->aen = aen;
}

void
bfa_fcs_modexit_comp(struct bfa_fcs_s *fcs)
{
	bfa_wc_down(&fcs->wc);
}


