

/* messages define for BFA_AEN_CAT_ITNIM Module */
#ifndef	__bfa_aen_itnim_h__
#define	__bfa_aen_itnim_h__

#include  <cs/bfa_log.h>
#include  <defs/bfa_defs_aen.h>

#define BFA_AEN_ITNIM_ONLINE \
	BFA_LOG_CREATE_ID(BFA_AEN_CAT_ITNIM, BFA_ITNIM_AEN_ONLINE)
#define BFA_AEN_ITNIM_OFFLINE \
	BFA_LOG_CREATE_ID(BFA_AEN_CAT_ITNIM, BFA_ITNIM_AEN_OFFLINE)
#define BFA_AEN_ITNIM_DISCONNECT \
	BFA_LOG_CREATE_ID(BFA_AEN_CAT_ITNIM, BFA_ITNIM_AEN_DISCONNECT)

#endif

