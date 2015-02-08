

#ifndef __IWM_BUS_H__
#define __IWM_BUS_H__

#include "iwm.h"

struct iwm_if_ops {
	int (*enable)(struct iwm_priv *iwm);
	int (*disable)(struct iwm_priv *iwm);
	int (*send_chunk)(struct iwm_priv *iwm, u8* buf, int count);

	void (*debugfs_init)(struct iwm_priv *iwm, struct dentry *parent_dir);
	void (*debugfs_exit)(struct iwm_priv *iwm);

	const char *umac_name;
	const char *calib_lmac_name;
	const char *lmac_name;
};

static inline int iwm_bus_send_chunk(struct iwm_priv *iwm, u8 *buf, int count)
{
	return iwm->bus_ops->send_chunk(iwm, buf, count);
}

static inline int iwm_bus_enable(struct iwm_priv *iwm)
{
	return iwm->bus_ops->enable(iwm);
}

static inline int iwm_bus_disable(struct iwm_priv *iwm)
{
	return iwm->bus_ops->disable(iwm);
}

#endif
