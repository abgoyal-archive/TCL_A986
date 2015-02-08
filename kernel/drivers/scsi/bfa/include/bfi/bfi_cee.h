

#ifndef __BFI_CEE_H__
#define __BFI_CEE_H__

#include <bfi/bfi.h>

#pragma pack(1)


enum bfi_cee_h2i_msgs_e {
	BFI_CEE_H2I_GET_CFG_REQ = 1,
	BFI_CEE_H2I_RESET_STATS = 2,
	BFI_CEE_H2I_GET_STATS_REQ = 3,
};


enum bfi_cee_i2h_msgs_e {
	BFI_CEE_I2H_GET_CFG_RSP = BFA_I2HM(1),
	BFI_CEE_I2H_RESET_STATS_RSP = BFA_I2HM(2),
	BFI_CEE_I2H_GET_STATS_RSP = BFA_I2HM(3),
};


/* Data structures */

struct bfi_lldp_reset_stats_s {
	struct bfi_mhdr_s  mh;
};

struct bfi_cee_reset_stats_s {
	struct bfi_mhdr_s  mh;
};

struct bfi_cee_get_req_s {
	struct bfi_mhdr_s  mh;
	union bfi_addr_u   dma_addr;
};


struct bfi_cee_get_rsp_s {
	struct bfi_mhdr_s  mh;
	u8            cmd_status;
	u8            rsvd[3];
};

struct bfi_cee_stats_req_s {
	struct bfi_mhdr_s  mh;
	union bfi_addr_u   dma_addr;
};


struct bfi_cee_stats_rsp_s {
	struct bfi_mhdr_s  mh;
	u8 		   cmd_status;
	u8 		   rsvd[3];
};



union bfi_cee_h2i_msg_u {
	struct bfi_mhdr_s           mh;
	struct bfi_cee_get_req_s   get_req;
	struct bfi_cee_stats_req_s stats_req;
};


union bfi_cee_i2h_msg_u {
	struct bfi_mhdr_s         mh;
	struct bfi_cee_get_rsp_s  get_rsp;
	struct bfi_cee_stats_rsp_s stats_rsp;
};

#pragma pack()


#endif /* __BFI_CEE_H__ */

