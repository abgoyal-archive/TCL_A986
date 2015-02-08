
/* @(#) $Header: /tcpdump/master/tcpdump/token.h,v 1.6 2002/12/11 07:14:12 guy Exp $ (LBL) */

#define TOKEN_HDRLEN		14
#define TOKEN_RING_MAC_LEN	6
#define ROUTING_SEGMENT_MAX	16
#define IS_SOURCE_ROUTED(trp)	((trp)->token_shost[0] & 0x80)
#define FRAME_TYPE(trp)		(((trp)->token_fc & 0xC0) >> 6)
#define TOKEN_FC_LLC		1

#define BROADCAST(trp)		((ntohs((trp)->token_rcf) & 0xE000) >> 13)
#define RIF_LENGTH(trp)		((ntohs((trp)->token_rcf) & 0x1f00) >> 8)
#define DIRECTION(trp)		((ntohs((trp)->token_rcf) & 0x0080) >> 7)
#define LARGEST_FRAME(trp)	((ntohs((trp)->token_rcf) & 0x0070) >> 4)
#define RING_NUMBER(trp, x)	((ntohs((trp)->token_rseg[x]) & 0xfff0) >> 4)
#define BRIDGE_NUMBER(trp, x)	((ntohs((trp)->token_rseg[x]) & 0x000f))
#define SEGMENT_COUNT(trp)	((int)((RIF_LENGTH(trp) - 2) / 2))

struct token_header {
	u_int8_t  token_ac;
	u_int8_t  token_fc;
	u_int8_t  token_dhost[TOKEN_RING_MAC_LEN];
	u_int8_t  token_shost[TOKEN_RING_MAC_LEN];
	u_int16_t token_rcf;
	u_int16_t token_rseg[ROUTING_SEGMENT_MAX];
};
