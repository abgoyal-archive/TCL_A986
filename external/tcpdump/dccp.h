
/* @(#) $Header: /tcpdump/master/tcpdump/dccp.h,v 1.1.2.4 2006/05/12 01:46:17 guy Exp $ (LBL) */

#ifndef __DCCP_HDR__
#define __DCCP_HDR__

struct dccp_hdr {
	u_int16_t	dccph_sport,
			dccph_dport;
	u_int8_t	dccph_doff;
	u_int8_t	dccph_ccval_cscov;
	u_int16_t	dccph_checksum;
	union {
	u_int8_t	dccph_xtr;
	u_int32_t	dccph_seq;
	}		dccph_xtrs;
};

#define DCCPH_CCVAL(dh)	(((dh)->dccph_ccval_cscov) & 0x0F)
#define DCCPH_CSCOV(dh)	(((dh)->dccph_ccval_cscov >> 4) & 0x0F)

#define DCCPH_X(dh)	((dh)->dccph_xtrs.dccph_xtr & 1)
#define DCCPH_TYPE(dh)	(((dh)->dccph_xtrs.dccph_xtr >> 1) & 0xF)
#define DCCPH_SEQ(dh)   (((dh)->dccph_xtrs.dccph_seq) >> 8)

struct dccp_hdr_ext {
	u_int32_t	dccph_seq_low;
};

struct dccp_hdr_request {
	u_int32_t	dccph_req_service;
};

struct dccp_hdr_ack_bits {
	u_int32_t	dccph_ra;
	u_int32_t	dccph_ack_nr_low;
};

#define DCCPH_ACK(dh_ack)   ((dh_ack)->dccph_ra >> 8)

struct dccp_hdr_response {
	struct dccp_hdr_ack_bits	dccph_resp_ack;
	u_int32_t			dccph_resp_service;
};

#if 0
static inline struct dccp_hdr_data *dccp_hdr_data(struct dccp_hdr *hdrg)
{
	const int ext = DCCPH_X(hdrg) ? sizeof(struct dccp_hdr_ext) : 0;

	return (struct dccp_hdr_data *)(((u_char *)hdrg) + sizeof(hdrg) + ext);
}
#endif

struct dccp_hdr_reset {
	struct dccp_hdr_ack_bits	dccph_reset_ack;
	u_int8_t			dccph_reset_code,
					dccph_reset_data[3];
};

enum dccp_pkt_type {
	DCCP_PKT_REQUEST = 0,
	DCCP_PKT_RESPONSE,
	DCCP_PKT_DATA,
	DCCP_PKT_ACK,
	DCCP_PKT_DATAACK,
	DCCP_PKT_CLOSEREQ,
	DCCP_PKT_CLOSE,
	DCCP_PKT_RESET,
	DCCP_PKT_SYNC,
	DCCP_PKT_SYNCACK,
	DCCP_PKT_INVALID
};

enum dccp_reset_codes {
	DCCP_RESET_CODE_UNSPECIFIED = 0,
	DCCP_RESET_CODE_CLOSED,
	DCCP_RESET_CODE_ABORTED,
	DCCP_RESET_CODE_NO_CONNECTION,
	DCCP_RESET_CODE_PACKET_ERROR,
	DCCP_RESET_CODE_OPTION_ERROR,
	DCCP_RESET_CODE_MANDATORY_ERROR,
	DCCP_RESET_CODE_CONNECTION_REFUSED,
	DCCP_RESET_CODE_BAD_SERVICE_CODE,
	DCCP_RESET_CODE_TOO_BUSY,
	DCCP_RESET_CODE_BAD_INIT_COOKIE,
	DCCP_RESET_CODE_AGGRESSION_PENALTY,
	__DCCP_RESET_CODE_LAST
};

#endif /* __DCCP_HDR__ */
