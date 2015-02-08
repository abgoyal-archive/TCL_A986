

#include "sas_internal.h"

#ifdef SAS_DEBUG

void sas_dprint_porte(int phyid, enum port_event pe);
void sas_dprint_phye(int phyid, enum phy_event pe);
void sas_dprint_hae(struct sas_ha_struct *sas_ha, enum ha_event he);
void sas_dump_port(struct asd_sas_port *port);

#else /* SAS_DEBUG */

static inline void sas_dprint_porte(int phyid, enum port_event pe) { }
static inline void sas_dprint_phye(int phyid, enum phy_event pe) { }
static inline void sas_dprint_hae(struct sas_ha_struct *sas_ha,
				  enum ha_event he) { }
static inline void sas_dump_port(struct asd_sas_port *port) { }

#endif /* SAS_DEBUG */
