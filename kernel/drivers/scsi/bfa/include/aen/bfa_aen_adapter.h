

/* messages define for BFA_AEN_CAT_ADAPTER Module */
#ifndef	__bfa_aen_adapter_h__
#define	__bfa_aen_adapter_h__

#include  <cs/bfa_log.h>
#include  <defs/bfa_defs_aen.h>

#define BFA_AEN_ADAPTER_ADD \
	BFA_LOG_CREATE_ID(BFA_AEN_CAT_ADAPTER, BFA_ADAPTER_AEN_ADD)
#define BFA_AEN_ADAPTER_REMOVE \
	BFA_LOG_CREATE_ID(BFA_AEN_CAT_ADAPTER, BFA_ADAPTER_AEN_REMOVE)

#endif

