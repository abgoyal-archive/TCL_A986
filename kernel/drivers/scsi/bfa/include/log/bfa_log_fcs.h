

#ifndef	__BFA_LOG_FCS_H__
#define	__BFA_LOG_FCS_H__
#include  <cs/bfa_log.h>
#define BFA_LOG_FCS_FABRIC_NOSWITCH 	\
	(((u32) BFA_LOG_FCS_ID << BFA_LOG_MODID_OFFSET) | 1)
#define BFA_LOG_FCS_FABRIC_ISOLATED 	\
	(((u32) BFA_LOG_FCS_ID << BFA_LOG_MODID_OFFSET) | 2)
#endif
