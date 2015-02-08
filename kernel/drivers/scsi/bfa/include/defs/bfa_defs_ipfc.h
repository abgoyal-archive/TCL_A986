
#ifndef __BFA_DEFS_IPFC_H__
#define __BFA_DEFS_IPFC_H__

#include <bfa_os_inc.h>
#include <protocol/types.h>
#include <defs/bfa_defs_types.h>

enum bfa_iprp_state {
	BFA_IPRP_UNINIT  = 0,	/*  PORT is not yet initialized */
	BFA_IPRP_ONLINE  = 1,	/*  process login is complete */
	BFA_IPRP_OFFLINE = 2,	/*  iprp is offline */
};

struct bfa_iprp_stats_s {
	u32        offlines;
	u32        onlines;
	u32        rscns;
	u32        plogis;
	u32        logos;
	u32        plogi_timeouts;
	u32        plogi_rejects;
};

struct bfa_iprp_attr_s {
	enum bfa_iprp_state state;
};

struct bfa_ipfc_stats_s {
	u32 arp_sent;
	u32 arp_recv;
	u32 arp_reply_sent;
	u32 arp_reply_recv;
	u32 farp_sent;
	u32 farp_recv;
	u32 farp_reply_sent;
	u32 farp_reply_recv;
	u32 farp_reject_sent;
	u32 farp_reject_recv;
};

struct bfa_ipfc_attr_s {
	bfa_boolean_t enabled;
};

#endif /* __BFA_DEFS_IPFC_H__ */
