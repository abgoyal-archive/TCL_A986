
#define LPFC_BSG_VENDOR_SET_CT_EVENT	1
#define LPFC_BSG_VENDOR_GET_CT_EVENT	2
#define LPFC_BSG_VENDOR_SEND_MGMT_RESP	3
#define LPFC_BSG_VENDOR_DIAG_MODE	4
#define LPFC_BSG_VENDOR_DIAG_TEST	5
#define LPFC_BSG_VENDOR_GET_MGMT_REV	6
#define LPFC_BSG_VENDOR_MBOX		7
#define LPFC_BSG_VENDOR_MENLO_CMD	8
#define LPFC_BSG_VENDOR_MENLO_DATA	9

struct set_ct_event {
	uint32_t command;
	uint32_t type_mask;
	uint32_t ev_req_id;
	uint32_t ev_reg_id;
};

struct get_ct_event {
	uint32_t command;
	uint32_t ev_reg_id;
	uint32_t ev_req_id;
};

struct get_ct_event_reply {
	uint32_t immed_data;
	uint32_t type;
};

struct send_mgmt_resp {
	uint32_t command;
	uint32_t tag;
};


#define INTERNAL_LOOP_BACK 0x1 /* adapter short cuts the loop internally */
#define EXTERNAL_LOOP_BACK 0x2 /* requires an external loopback plug */

struct diag_mode_set {
	uint32_t command;
	uint32_t type;
	uint32_t timeout;
};

struct diag_mode_test {
	uint32_t command;
};

#define LPFC_WWNN_TYPE		0
#define LPFC_WWPN_TYPE		1

struct get_mgmt_rev {
	uint32_t command;
};

#define MANAGEMENT_MAJOR_REV   1
#define MANAGEMENT_MINOR_REV   0

/* the MgmtRevInfo structure */
struct MgmtRevInfo {
	uint32_t a_Major;
	uint32_t a_Minor;
};

struct get_mgmt_rev_reply {
	struct MgmtRevInfo info;
};

#define BSG_MBOX_SIZE 4096 /* mailbox command plus extended data */
struct dfc_mbox_req {
	uint32_t command;
	uint32_t mbOffset;
	uint32_t inExtWLen;
	uint32_t outExtWLen;
};

/* Used for menlo command or menlo data. The xri is only used for menlo data */
struct menlo_command {
	uint32_t cmd;
	uint32_t xri;
};

struct menlo_response {
	uint32_t xri; /* return the xri of the iocb exchange */
};

