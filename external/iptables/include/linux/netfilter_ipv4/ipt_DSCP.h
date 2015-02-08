
#ifndef _IPT_DSCP_TARGET_H
#define _IPT_DSCP_TARGET_H
#include <linux/netfilter_ipv4/ipt_dscp.h>

/* target info */
struct ipt_DSCP_info {
	u_int8_t dscp;
};

#endif /* _IPT_DSCP_TARGET_H */
