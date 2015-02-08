

#ifndef __BFI_FCPIM_H__
#define __BFI_FCPIM_H__

#include "bfi.h"
#include <protocol/fcp.h>

#pragma pack(1)


enum bfi_itnim_h2i {
	BFI_ITNIM_H2I_CREATE_REQ = 1,	/*  i-t nexus creation */
	BFI_ITNIM_H2I_DELETE_REQ = 2,	/*  i-t nexus deletion */
};

enum bfi_itnim_i2h {
	BFI_ITNIM_I2H_CREATE_RSP = BFA_I2HM(1),
	BFI_ITNIM_I2H_DELETE_RSP = BFA_I2HM(2),
	BFI_ITNIM_I2H_SLER_EVENT = BFA_I2HM(3),
};

struct bfi_itnim_create_req_s {
	struct bfi_mhdr_s  mh;		/*  common msg header		 */
	u16        fw_handle;	/*  f/w handle for itnim	 */
	u8         class;		/*  FC class for IO		 */
	u8         seq_rec;	/*  sequence recovery support	 */
	u8         msg_no;		/*  seq id of the msg		 */
};

struct bfi_itnim_create_rsp_s {
	struct bfi_mhdr_s  mh;		/*  common msg header		 */
	u16        bfa_handle;	/*  bfa handle for itnim	 */
	u8         status;		/*  fcp request status		 */
	u8         seq_id;		/*  seq id of the msg		 */
};

struct bfi_itnim_delete_req_s {
	struct bfi_mhdr_s  mh;		/*  common msg header		 */
	u16        fw_handle;	/*  f/w itnim handle		 */
	u8         seq_id;		/*  seq id of the msg		 */
	u8         rsvd;
};

struct bfi_itnim_delete_rsp_s {
	struct bfi_mhdr_s  mh;		/*  common msg header		 */
	u16        bfa_handle;	/*  bfa handle for itnim	 */
	u8         status;		/*  fcp request status		 */
	u8         seq_id;		/*  seq id of the msg		 */
};

struct bfi_itnim_sler_event_s {
	struct bfi_mhdr_s  mh;		/*  common msg header		 */
	u16        bfa_handle;	/*  bfa handle for itnim	 */
	u16        rsvd;
};

union bfi_itnim_h2i_msg_u {
	struct bfi_itnim_create_req_s *create_req;
	struct bfi_itnim_delete_req_s *delete_req;
	struct bfi_msg_s      *msg;
};

union bfi_itnim_i2h_msg_u {
	struct bfi_itnim_create_rsp_s *create_rsp;
	struct bfi_itnim_delete_rsp_s *delete_rsp;
	struct bfi_itnim_sler_event_s *sler_event;
	struct bfi_msg_s      *msg;
};


enum bfi_ioim_h2i {
	BFI_IOIM_H2I_IOABORT_REQ = 1,	/*  IO abort request	 */
	BFI_IOIM_H2I_IOCLEANUP_REQ = 2,	/*  IO cleanup request	 */
};

enum bfi_ioim_i2h {
	BFI_IOIM_I2H_IO_RSP = BFA_I2HM(1),	/*  non-fp IO response	 */
	BFI_IOIM_I2H_IOABORT_RSP = BFA_I2HM(2),/*  ABORT rsp	 */
};

struct bfi_ioim_dif_s {
	u32	dif_info[4];
};

struct bfi_ioim_req_s {
	struct bfi_mhdr_s  mh;		/*  Common msg header		 */
	u16        io_tag;		/*  I/O tag			 */
	u16        rport_hdl;	/*  itnim/rport firmware handle */
	struct fcp_cmnd_s cmnd;		/*  IO request info		 */

	/**
	 * SG elements array within the IO request must be double word
	 * aligned. This aligment is required to optimize SGM setup for the IO.
	 */
	struct bfi_sge_s   sges[BFI_SGE_INLINE_MAX];
	u8         io_timeout;
	u8         dif_en;
	u8         rsvd_a[2];
	struct bfi_ioim_dif_s  dif;
};

enum bfi_ioim_status {
	BFI_IOIM_STS_OK = 0,
	BFI_IOIM_STS_HOST_ABORTED = 1,
	BFI_IOIM_STS_ABORTED = 2,
	BFI_IOIM_STS_TIMEDOUT = 3,
	BFI_IOIM_STS_RES_FREE = 4,
	BFI_IOIM_STS_SQER_NEEDED = 5,
	BFI_IOIM_STS_PROTO_ERR = 6,
	BFI_IOIM_STS_UTAG = 7,
	BFI_IOIM_STS_PATHTOV = 8,
};

#define BFI_IOIM_SNSLEN	(256)
struct bfi_ioim_rsp_s {
	struct bfi_mhdr_s  mh;		/*  common msg header	 	 */
	u16        io_tag;		/*  completed IO tag		 */
	u16        bfa_rport_hndl;	/*  releated rport handle	 */
	u8         io_status;	/*  IO completion status	 */
	u8         reuse_io_tag;	/*  IO tag can be reused        */
	u16	abort_tag;	/*  host abort request tag      */
	u8		scsi_status;	/*  scsi status from target	 */
	u8		sns_len;	/*  scsi sense length		 */
	u8		resid_flags;	/*  IO residue flags		 */
	u8		rsvd_a;
	u32	residue;	/*  IO residual length in bytes */
	u32	rsvd_b[3];
};

struct bfi_ioim_abort_req_s {
	struct bfi_mhdr_s  mh;          /*  Common msg header  */
	u16        io_tag;         /*  I/O tag            */
	u16        abort_tag;      /*  unique request tag */
};


enum bfi_tskim_h2i {
	BFI_TSKIM_H2I_TM_REQ    = 1, /*  task-mgmt command         */
	BFI_TSKIM_H2I_ABORT_REQ = 2, /*  task-mgmt command         */
};

enum bfi_tskim_i2h {
	BFI_TSKIM_I2H_TM_RSP = BFA_I2HM(1),
};

struct bfi_tskim_req_s {
	struct bfi_mhdr_s  mh;             /*  Common msg header          */
	u16        tsk_tag;        /*  task management tag        */
	u16        itn_fhdl;       /*  itn firmware handle        */
	lun_t           lun;            /*  LU number                  */
	u8         tm_flags;       /*  see fcp_tm_cmnd_t          */
	u8         t_secs;         /*  Timeout value in seconds   */
	u8         rsvd[2];
};

struct bfi_tskim_abortreq_s {
	struct bfi_mhdr_s  mh;             /*  Common msg header          */
	u16        tsk_tag;        /*  task management tag        */
	u16        rsvd;
};

enum bfi_tskim_status {
	/*
	 * Following are FCP-4 spec defined status codes,
	 * **DO NOT CHANGE THEM **
	 */
	BFI_TSKIM_STS_OK       = 0,
	BFI_TSKIM_STS_NOT_SUPP = 4,
	BFI_TSKIM_STS_FAILED   = 5,

	/**
	 * Defined by BFA
	 */
	BFI_TSKIM_STS_TIMEOUT  = 10,    /*  TM request timedout     */
	BFI_TSKIM_STS_ABORTED  = 11,    /*  Aborted on host request */
};

struct bfi_tskim_rsp_s {
	struct bfi_mhdr_s  mh;		/*  Common msg header		 */
	u16        tsk_tag;	/*  task mgmt cmnd tag		 */
	u8         tsk_status;	/*  @ref bfi_tskim_status */
	u8         rsvd;
};

#pragma pack()

#endif /* __BFI_FCPIM_H__ */

