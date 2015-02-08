

/* messages define for BFA_AEN_CAT_IOC Module */
#ifndef	__bfa_aen_ioc_h__
#define	__bfa_aen_ioc_h__

#include  <cs/bfa_log.h>
#include  <defs/bfa_defs_aen.h>

#define BFA_AEN_IOC_HBGOOD \
	BFA_LOG_CREATE_ID(BFA_AEN_CAT_IOC, BFA_IOC_AEN_HBGOOD)
#define BFA_AEN_IOC_HBFAIL \
	BFA_LOG_CREATE_ID(BFA_AEN_CAT_IOC, BFA_IOC_AEN_HBFAIL)
#define BFA_AEN_IOC_ENABLE \
	BFA_LOG_CREATE_ID(BFA_AEN_CAT_IOC, BFA_IOC_AEN_ENABLE)
#define BFA_AEN_IOC_DISABLE \
	BFA_LOG_CREATE_ID(BFA_AEN_CAT_IOC, BFA_IOC_AEN_DISABLE)
#define BFA_AEN_IOC_FWMISMATCH \
	BFA_LOG_CREATE_ID(BFA_AEN_CAT_IOC, BFA_IOC_AEN_FWMISMATCH)

#endif

