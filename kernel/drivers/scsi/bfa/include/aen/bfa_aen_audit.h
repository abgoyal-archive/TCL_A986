

/* messages define for BFA_AEN_CAT_AUDIT Module */
#ifndef	__bfa_aen_audit_h__
#define	__bfa_aen_audit_h__

#include  <cs/bfa_log.h>
#include  <defs/bfa_defs_aen.h>

#define BFA_AEN_AUDIT_AUTH_ENABLE \
	BFA_LOG_CREATE_ID(BFA_AEN_CAT_AUDIT, BFA_AUDIT_AEN_AUTH_ENABLE)
#define BFA_AEN_AUDIT_AUTH_DISABLE \
	BFA_LOG_CREATE_ID(BFA_AEN_CAT_AUDIT, BFA_AUDIT_AEN_AUTH_DISABLE)

#endif

