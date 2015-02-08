
#ifndef _IPT_ECN_H
#define _IPT_ECN_H
#include <linux/netfilter_ipv4/ipt_dscp_.h>

#define IPT_ECN_IP_MASK	(~IPT_DSCP_MASK)

#define IPT_ECN_OP_MATCH_IP	0x01
#define IPT_ECN_OP_MATCH_ECE	0x10
#define IPT_ECN_OP_MATCH_CWR	0x20

#define IPT_ECN_OP_MATCH_MASK	0xce

/* match info */
struct ipt_ecn_info {
	u_int8_t operation;
	u_int8_t invert;
	u_int8_t ip_ect;
	union {
		struct {
			u_int8_t ect;
		} tcp;
	} proto;
};

#endif /* _IPT_ECN_H */
