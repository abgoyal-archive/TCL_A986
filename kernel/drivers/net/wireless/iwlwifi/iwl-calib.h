
#ifndef __iwl_calib_h__
#define __iwl_calib_h__

#include "iwl-dev.h"
#include "iwl-core.h"
#include "iwl-commands.h"

void iwl_chain_noise_calibration(struct iwl_priv *priv,
				struct iwl_notif_statistics *stat_resp);
void iwl_sensitivity_calibration(struct iwl_priv *priv,
				struct iwl_notif_statistics *resp);

void iwl_init_sensitivity(struct iwl_priv *priv);
void iwl_reset_run_time_calib(struct iwl_priv *priv);
static inline void iwl_chain_noise_reset(struct iwl_priv *priv)
{

	if (!priv->disable_chain_noise_cal &&
	    priv->cfg->ops->utils->chain_noise_reset)
		priv->cfg->ops->utils->chain_noise_reset(priv);
}

#endif /* __iwl_calib_h__ */
