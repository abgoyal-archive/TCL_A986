#ifndef _IPT_DSCP_H
#define _IPT_DSCP_H

#define IPT_DSCP_MASK	0xfc	/* 11111100 */
#define IPT_DSCP_SHIFT	2
#define IPT_DSCP_MAX	0x3f	/* 00111111 */

/* match info */
struct ipt_dscp_info {
	u_int8_t dscp;
	u_int8_t invert;
};

#endif /* _IPT_DSCP_H */
