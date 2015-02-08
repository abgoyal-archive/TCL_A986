

#ifndef __BFI_LPS_H__
#define __BFI_LPS_H__

#include <bfi/bfi.h>

#pragma pack(1)

enum bfi_lps_h2i_msgs {
	BFI_LPS_H2I_LOGIN_REQ	= 1,
	BFI_LPS_H2I_LOGOUT_REQ	= 2,
};

enum bfi_lps_i2h_msgs {
	BFI_LPS_H2I_LOGIN_RSP	= BFA_I2HM(1),
	BFI_LPS_H2I_LOGOUT_RSP	= BFA_I2HM(2),
	BFI_LPS_H2I_CVL_EVENT   = BFA_I2HM(3),
};

struct bfi_lps_login_req_s {
	struct bfi_mhdr_s  mh;		/*  common msg header		*/
	u8		lp_tag;
	u8		alpa;
	u16	pdu_size;
	wwn_t		pwwn;
	wwn_t		nwwn;
	u8		fdisc;
	u8		auth_en;
	u8		rsvd[2];
};

struct bfi_lps_login_rsp_s {
	struct bfi_mhdr_s  mh;		/*  common msg header		*/
	u8		lp_tag;
	u8		status;
	u8		lsrjt_rsn;
	u8		lsrjt_expl;
	wwn_t		port_name;
	wwn_t		node_name;
	u16	bb_credit;
	u8		f_port;
	u8		npiv_en;
	u32	lp_pid:24;
	u32	auth_req:8;
	mac_t		lp_mac;
	mac_t		fcf_mac;
	u8		ext_status;
	u8  	brcd_switch;/*  attached peer is brcd switch	*/
};

struct bfi_lps_logout_req_s {
	struct bfi_mhdr_s  mh;		/*  common msg header		*/
	u8		lp_tag;
	u8		rsvd[3];
	wwn_t		port_name;
};

struct bfi_lps_logout_rsp_s {
	struct bfi_mhdr_s  mh;		/*  common msg header		*/
	u8		lp_tag;
	u8		status;
	u8		rsvd[2];
};

struct bfi_lps_cvl_event_s {
	struct bfi_mhdr_s  mh;      /* common msg header      */
	u8		lp_tag;
	u8		rsvd[3];
};

union bfi_lps_h2i_msg_u {
	struct bfi_mhdr_s		*msg;
	struct bfi_lps_login_req_s	*login_req;
	struct bfi_lps_logout_req_s	*logout_req;
};

union bfi_lps_i2h_msg_u {
	struct bfi_msg_s		*msg;
	struct bfi_lps_login_rsp_s	*login_rsp;
	struct bfi_lps_logout_rsp_s	*logout_rsp;
	struct bfi_lps_cvl_event_s	*cvl_event;
};

#pragma pack()

#endif /* __BFI_LPS_H__ */


