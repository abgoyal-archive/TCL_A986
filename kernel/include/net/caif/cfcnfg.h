

#ifndef CFCNFG_H_
#define CFCNFG_H_
#include <linux/spinlock.h>
#include <net/caif/caif_layer.h>
#include <net/caif/cfctrl.h>

struct cfcnfg;

enum cfcnfg_phy_type {
	CFPHYTYPE_FRAG = 1,
	CFPHYTYPE_CAIF,
	CFPHYTYPE_MAX
};

enum cfcnfg_phy_preference {
	CFPHYPREF_UNSPECIFIED,
	CFPHYPREF_LOW_LAT,
	CFPHYPREF_HIGH_BW,
	CFPHYPREF_LOOP
};

struct cfcnfg *cfcnfg_create(void);

void cfcnfg_remove(struct cfcnfg *cfg);


void
cfcnfg_add_phy_layer(struct cfcnfg *cnfg, enum cfcnfg_phy_type phy_type,
		     void *dev, struct cflayer *phy_layer, u16 *phyid,
		     enum cfcnfg_phy_preference pref,
		     bool fcs, bool stx);

int cfcnfg_del_phy_layer(struct cfcnfg *cnfg, struct cflayer *phy_layer);

int cfcnfg_disconn_adapt_layer(struct cfcnfg *cnfg,
			struct cflayer *adap_layer);

void cfcnfg_release_adap_layer(struct cflayer *adap_layer);

int cfcnfg_add_adaptation_layer(struct cfcnfg *cnfg,
			    struct cfctrl_link_param *param,
			    struct cflayer *adap_layer);

struct dev_info *cfcnfg_get_phyid(struct cfcnfg *cnfg,
		     enum cfcnfg_phy_preference phy_pref);

int cfcnfg_get_named(struct cfcnfg *cnfg, char *name);

#endif				/* CFCNFG_H_ */
