

#define FC_MAX_HOLD_RSCN     32	      /* max number of deferred RSCNs */
#define FC_MAX_NS_RSP        64512    /* max size NameServer rsp */
#define FC_MAXLOOP           126      /* max devices supported on a fc loop */
#define LPFC_DISC_FLOGI_TMO  10	      /* Discovery FLOGI ratov */



/* worker thread events */
enum lpfc_work_type {
	LPFC_EVT_ONLINE,
	LPFC_EVT_OFFLINE_PREP,
	LPFC_EVT_OFFLINE,
	LPFC_EVT_WARM_START,
	LPFC_EVT_KILL,
	LPFC_EVT_ELS_RETRY,
	LPFC_EVT_DEV_LOSS,
	LPFC_EVT_FASTPATH_MGMT_EVT,
	LPFC_EVT_RESET_HBA,
};

/* structure used to queue event to the discovery tasklet */
struct lpfc_work_evt {
	struct list_head      evt_listp;
	void                 *evt_arg1;
	void                 *evt_arg2;
	enum lpfc_work_type   evt;
};

struct lpfc_scsi_check_condition_event;
struct lpfc_scsi_varqueuedepth_event;
struct lpfc_scsi_event_header;
struct lpfc_fabric_event_header;
struct lpfc_fcprdchkerr_event;

/* structure used for sending events from fast path */
struct lpfc_fast_path_event {
	struct lpfc_work_evt work_evt;
	struct lpfc_vport     *vport;
	union {
		struct lpfc_scsi_check_condition_event check_cond_evt;
		struct lpfc_scsi_varqueuedepth_event queue_depth_evt;
		struct lpfc_scsi_event_header scsi_evt;
		struct lpfc_fabric_event_header fabric_evt;
		struct lpfc_fcprdchkerr_event read_check_error;
	} un;
};

struct lpfc_nodelist {
	struct list_head nlp_listp;
	struct lpfc_name nlp_portname;
	struct lpfc_name nlp_nodename;
	uint32_t         nlp_flag;		/* entry  flags */
	uint32_t         nlp_DID;		/* FC D_ID of entry */
	uint32_t         nlp_last_elscmd;	/* Last ELS cmd sent */
	uint16_t         nlp_type;
#define NLP_FC_NODE        0x1			/* entry is an FC node */
#define NLP_FABRIC         0x4			/* entry rep a Fabric entity */
#define NLP_FCP_TARGET     0x8			/* entry is an FCP target */
#define NLP_FCP_INITIATOR  0x10			/* entry is an FCP Initiator */

	uint16_t        nlp_rpi;
	uint16_t        nlp_state;		/* state transition indicator */
	uint16_t        nlp_prev_state;		/* state transition indicator */
	uint16_t        nlp_xri;		/* output exchange id for RPI */
	uint16_t        nlp_sid;		/* scsi id */
#define NLP_NO_SID		0xffff
	uint16_t	nlp_maxframe;		/* Max RCV frame size */
	uint8_t		nlp_class_sup;		/* Supported Classes */
	uint8_t         nlp_retry;		/* used for ELS retries */
	uint8_t         nlp_fcp_info;	        /* class info, bits 0-3 */
#define NLP_FCP_2_DEVICE   0x10			/* FCP-2 device */

	uint16_t        nlp_usg_map;	/* ndlp management usage bitmap */
#define NLP_USG_NODE_ACT_BIT	0x1	/* Indicate ndlp is actively used */
#define NLP_USG_IACT_REQ_BIT	0x2	/* Request to inactivate ndlp */
#define NLP_USG_FREE_REQ_BIT	0x4	/* Request to invoke ndlp memory free */
#define NLP_USG_FREE_ACK_BIT	0x8	/* Indicate ndlp memory free invoked */

	struct timer_list   nlp_delayfunc;	/* Used for delayed ELS cmds */
	struct lpfc_hba *phba;
	struct fc_rport *rport;			/* Corresponding FC transport
						   port structure */
	struct lpfc_vport *vport;
	struct lpfc_work_evt els_retry_evt;
	struct lpfc_work_evt dev_loss_evt;
	struct kref     kref;
	atomic_t cmd_pending;
	uint32_t cmd_qdepth;
	unsigned long last_change_time;
	struct lpfc_scsicmd_bkt *lat_data;	/* Latency data */
};

/* Defines for nlp_flag (uint32) */
#define NLP_PLOGI_SND      0x00000020	/* sent PLOGI request for this entry */
#define NLP_PRLI_SND       0x00000040	/* sent PRLI request for this entry */
#define NLP_ADISC_SND      0x00000080	/* sent ADISC request for this entry */
#define NLP_LOGO_SND       0x00000100	/* sent LOGO request for this entry */
#define NLP_RNID_SND       0x00000400	/* sent RNID request for this entry */
#define NLP_ELS_SND_MASK   0x000007e0	/* sent ELS request for this entry */
#define NLP_DEFER_RM       0x00010000	/* Remove this ndlp if no longer used */
#define NLP_DELAY_TMO      0x00020000	/* delay timeout is running for node */
#define NLP_NPR_2B_DISC    0x00040000	/* node is included in num_disc_nodes */
#define NLP_RCV_PLOGI      0x00080000	/* Rcv'ed PLOGI from remote system */
#define NLP_LOGO_ACC       0x00100000	/* Process LOGO after ACC completes */
#define NLP_TGT_NO_SCSIID  0x00200000	/* good PRLI but no binding for scsid */
#define NLP_ACC_REGLOGIN   0x01000000	/* Issue Reg Login after successful
					   ACC */
#define NLP_NPR_ADISC      0x02000000	/* Issue ADISC when dq'ed from
					   NPR list */
#define NLP_RM_DFLT_RPI    0x04000000	/* need to remove leftover dflt RPI */
#define NLP_NODEV_REMOVE   0x08000000	/* Defer removal till discovery ends */
#define NLP_TARGET_REMOVE  0x10000000   /* Target remove in process */
#define NLP_SC_REQ         0x20000000	/* Target requires authentication */
#define NLP_RPI_VALID      0x80000000	/* nlp_rpi is valid */

/* ndlp usage management macros */
#define NLP_CHK_NODE_ACT(ndlp)		(((ndlp)->nlp_usg_map \
						& NLP_USG_NODE_ACT_BIT) \
					&& \
					!((ndlp)->nlp_usg_map \
						& NLP_USG_FREE_ACK_BIT))
#define NLP_SET_NODE_ACT(ndlp)		((ndlp)->nlp_usg_map \
						|= NLP_USG_NODE_ACT_BIT)
#define NLP_INT_NODE_ACT(ndlp)		((ndlp)->nlp_usg_map \
						= NLP_USG_NODE_ACT_BIT)
#define NLP_CLR_NODE_ACT(ndlp)		((ndlp)->nlp_usg_map \
						&= ~NLP_USG_NODE_ACT_BIT)
#define NLP_CHK_IACT_REQ(ndlp)          ((ndlp)->nlp_usg_map \
						& NLP_USG_IACT_REQ_BIT)
#define NLP_SET_IACT_REQ(ndlp)          ((ndlp)->nlp_usg_map \
						|= NLP_USG_IACT_REQ_BIT)
#define NLP_CHK_FREE_REQ(ndlp)		((ndlp)->nlp_usg_map \
						& NLP_USG_FREE_REQ_BIT)
#define NLP_SET_FREE_REQ(ndlp)		((ndlp)->nlp_usg_map \
						|= NLP_USG_FREE_REQ_BIT)
#define NLP_CHK_FREE_ACK(ndlp)		((ndlp)->nlp_usg_map \
						& NLP_USG_FREE_ACK_BIT)
#define NLP_SET_FREE_ACK(ndlp)		((ndlp)->nlp_usg_map \
						|= NLP_USG_FREE_ACK_BIT)


/* Defines for nlp_state */
#define NLP_STE_UNUSED_NODE       0x0	/* node is just allocated */
#define NLP_STE_PLOGI_ISSUE       0x1	/* PLOGI was sent to NL_PORT */
#define NLP_STE_ADISC_ISSUE       0x2	/* ADISC was sent to NL_PORT */
#define NLP_STE_REG_LOGIN_ISSUE   0x3	/* REG_LOGIN was issued for NL_PORT */
#define NLP_STE_PRLI_ISSUE        0x4	/* PRLI was sent to NL_PORT */
#define NLP_STE_UNMAPPED_NODE     0x5	/* PRLI completed from NL_PORT */
#define NLP_STE_MAPPED_NODE       0x6	/* Identified as a FCP Target */
#define NLP_STE_NPR_NODE          0x7	/* NPort disappeared */
#define NLP_STE_MAX_STATE         0x8
#define NLP_STE_FREED_NODE        0xff	/* node entry was freed to MEM_NLP */


/* Defines for Node List Entry Events that could happen */
#define NLP_EVT_RCV_PLOGI         0x0	/* Rcv'd an ELS PLOGI command */
#define NLP_EVT_RCV_PRLI          0x1	/* Rcv'd an ELS PRLI  command */
#define NLP_EVT_RCV_LOGO          0x2	/* Rcv'd an ELS LOGO  command */
#define NLP_EVT_RCV_ADISC         0x3	/* Rcv'd an ELS ADISC command */
#define NLP_EVT_RCV_PDISC         0x4	/* Rcv'd an ELS PDISC command */
#define NLP_EVT_RCV_PRLO          0x5	/* Rcv'd an ELS PRLO  command */
#define NLP_EVT_CMPL_PLOGI        0x6	/* Sent an ELS PLOGI command */
#define NLP_EVT_CMPL_PRLI         0x7	/* Sent an ELS PRLI  command */
#define NLP_EVT_CMPL_LOGO         0x8	/* Sent an ELS LOGO  command */
#define NLP_EVT_CMPL_ADISC        0x9	/* Sent an ELS ADISC command */
#define NLP_EVT_CMPL_REG_LOGIN    0xa	/* REG_LOGIN mbox cmd completed */
#define NLP_EVT_DEVICE_RM         0xb	/* Device not found in NS / ALPAmap */
#define NLP_EVT_DEVICE_RECOVERY   0xc	/* Device existence unknown */
#define NLP_EVT_MAX_EVENT         0xd

