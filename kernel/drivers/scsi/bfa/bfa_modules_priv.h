

#ifndef __BFA_MODULES_PRIV_H__
#define __BFA_MODULES_PRIV_H__

#include "bfa_uf_priv.h"
#include "bfa_port_priv.h"
#include "bfa_rport_priv.h"
#include "bfa_fcxp_priv.h"
#include "bfa_lps_priv.h"
#include "bfa_fcpim_priv.h"
#include <cee/bfa_cee.h>
#include <port/bfa_port.h>


struct bfa_modules_s {
	struct bfa_fcport_s	fcport;	/*  fc port module	*/
	struct bfa_fcxp_mod_s fcxp_mod; /*  fcxp module		*/
	struct bfa_lps_mod_s lps_mod;   /*  fcxp module		*/
	struct bfa_uf_mod_s uf_mod;	/*  unsolicited frame module	*/
	struct bfa_rport_mod_s rport_mod; /*  remote port module	*/
	struct bfa_fcpim_mod_s fcpim_mod; /*  FCP initiator module	*/
	struct bfa_sgpg_mod_s sgpg_mod; /*  SG page module		*/
	struct bfa_cee_s cee;   	/*  CEE Module                 */
	struct bfa_port_s port;		/*  Physical port module	*/
};

#endif /* __BFA_MODULES_PRIV_H__ */
