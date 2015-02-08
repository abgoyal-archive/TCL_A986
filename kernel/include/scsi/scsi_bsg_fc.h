

#ifndef SCSI_BSG_FC_H
#define SCSI_BSG_FC_H


#include <scsi/scsi.h>


/* Default BSG request timeout (in seconds) */
#define FC_DEFAULT_BSG_TIMEOUT		(10 * HZ)



/* define the class masks for the message codes */
#define FC_BSG_CLS_MASK		0xF0000000	/* find object class */
#define FC_BSG_HST_MASK		0x80000000	/* fc host class */
#define FC_BSG_RPT_MASK		0x40000000	/* fc rport class */

	/* fc_host Message Codes */
#define FC_BSG_HST_ADD_RPORT		(FC_BSG_HST_MASK | 0x00000001)
#define FC_BSG_HST_DEL_RPORT		(FC_BSG_HST_MASK | 0x00000002)
#define FC_BSG_HST_ELS_NOLOGIN		(FC_BSG_HST_MASK | 0x00000003)
#define FC_BSG_HST_CT			(FC_BSG_HST_MASK | 0x00000004)
#define FC_BSG_HST_VENDOR		(FC_BSG_HST_MASK | 0x000000FF)

	/* fc_rport Message Codes */
#define FC_BSG_RPT_ELS			(FC_BSG_RPT_MASK | 0x00000001)
#define FC_BSG_RPT_CT			(FC_BSG_RPT_MASK | 0x00000002)






/* FC_BSG_HST_ADDR_PORT : */

struct fc_bsg_host_add_rport {
	uint8_t		reserved;

	/* FC Address Identier of the remote port to login to */
	uint8_t		port_id[3];
};



/* FC_BSG_HST_DEL_RPORT : */

struct fc_bsg_host_del_rport {
	uint8_t		reserved;

	/* FC Address Identier of the remote port to logout of */
	uint8_t		port_id[3];
};



/* FC_BSG_HST_ELS_NOLOGIN : */

struct fc_bsg_host_els {
	/*
	 * ELS Command Code being sent (must be the same as byte 0
	 * of the payload)
	 */
	uint8_t 	command_code;

	/* FC Address Identier of the remote port to send the ELS to */
	uint8_t		port_id[3];
};

/* fc_bsg_ctels_reply->status values */
#define FC_CTELS_STATUS_OK	0x00000000
#define FC_CTELS_STATUS_REJECT	0x00000001
#define FC_CTELS_STATUS_P_RJT	0x00000002
#define FC_CTELS_STATUS_F_RJT	0x00000003
#define FC_CTELS_STATUS_P_BSY	0x00000004
#define FC_CTELS_STATUS_F_BSY	0x00000006
struct fc_bsg_ctels_reply {
	/*
	 * Note: An ELS LS_RJT may be reported in 2 ways:
	 *  a) A status of FC_CTELS_STATUS_OK is returned. The caller
	 *     is to look into the ELS receive payload to determine
	 *     LS_ACC or LS_RJT (by contents of word 0). The reject
	 *     data will be in word 1.
	 *  b) A status of FC_CTELS_STATUS_REJECT is returned, The
	 *     rjt_data field will contain valid data.
	 *
	 * Note: ELS LS_ACC is determined by an FC_CTELS_STATUS_OK, and
	 *   the receive payload word 0 indicates LS_ACC
	 *   (e.g. value is 0x02xxxxxx).
	 *
	 * Note: Similarly, a CT Reject may be reported in 2 ways:
	 *  a) A status of FC_CTELS_STATUS_OK is returned. The caller
	 *     is to look into the CT receive payload to determine
	 *     Accept or Reject (by contents of word 2). The reject
	 *     data will be in word 3.
	 *  b) A status of FC_CTELS_STATUS_REJECT is returned, The
	 *     rjt_data field will contain valid data.
	 *
	 * Note: x_RJT/BSY status will indicae that the rjt_data field
	 *   is valid and contains the reason/explanation values.
	 */
	uint32_t	status;		/* See FC_CTELS_STATUS_xxx */

	/* valid if status is not FC_CTELS_STATUS_OK */
	struct	{
		uint8_t	action;		/* fragment_id for CT REJECT */
		uint8_t	reason_code;
		uint8_t	reason_explanation;
		uint8_t	vendor_unique;
	} rjt_data;
};


/* FC_BSG_HST_CT : */

struct fc_bsg_host_ct {
	uint8_t		reserved;

	/* FC Address Identier of the remote port to send the ELS to */
	uint8_t		port_id[3];

	/*
	 * We need words 0-2 of the generic preamble for the LLD's
	 */
	uint32_t	preamble_word0;	/* revision & IN_ID */
	uint32_t	preamble_word1;	/* GS_Type, GS_SubType, Options, Rsvd */
	uint32_t	preamble_word2;	/* Cmd Code, Max Size */

};


/* FC_BSG_HST_VENDOR : */

struct fc_bsg_host_vendor {
	/*
	 * Identifies the vendor that the message is formatted for. This
	 * should be the recipient of the message.
	 */
	uint64_t vendor_id;

	/* start of vendor command area */
	uint32_t vendor_cmd[0];
};

struct fc_bsg_host_vendor_reply {
	/* start of vendor response area */
	uint32_t vendor_rsp[0];
};




/* FC_BSG_RPT_ELS : */

struct fc_bsg_rport_els {
	/*
	 * ELS Command Code being sent (must be the same as
	 * byte 0 of the payload)
	 */
	uint8_t els_code;
};



/* FC_BSG_RPT_CT : */

struct fc_bsg_rport_ct {
	/*
	 * We need words 0-2 of the generic preamble for the LLD's
	 */
	uint32_t	preamble_word0;	/* revision & IN_ID */
	uint32_t	preamble_word1;	/* GS_Type, GS_SubType, Options, Rsvd */
	uint32_t	preamble_word2;	/* Cmd Code, Max Size */
};




/* request (CDB) structure of the sg_io_v4 */
struct fc_bsg_request {
	uint32_t msgcode;
	union {
		struct fc_bsg_host_add_rport	h_addrport;
		struct fc_bsg_host_del_rport	h_delrport;
		struct fc_bsg_host_els		h_els;
		struct fc_bsg_host_ct		h_ct;
		struct fc_bsg_host_vendor	h_vendor;

		struct fc_bsg_rport_els		r_els;
		struct fc_bsg_rport_ct		r_ct;
	} rqst_data;
} __attribute__((packed));


/* response (request sense data) structure of the sg_io_v4 */
struct fc_bsg_reply {
	/*
	 * The completion result. Result exists in two forms:
	 *  if negative, it is an -Exxx system errno value. There will
	 *    be no further reply information supplied.
	 *  else, it's the 4-byte scsi error result, with driver, host,
	 *    msg and status fields. The per-msgcode reply structure
	 *    will contain valid data.
	 */
	uint32_t result;

	/* If there was reply_payload, how much was recevied ? */
	uint32_t reply_payload_rcv_len;

	union {
		struct fc_bsg_host_vendor_reply		vendor_reply;

		struct fc_bsg_ctels_reply		ctels_reply;
	} reply_data;
};


#endif /* SCSI_BSG_FC_H */

