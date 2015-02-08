

#ifndef _H_LPFC_DEBUG_FS
#define _H_LPFC_DEBUG_FS

#ifdef CONFIG_SCSI_LPFC_DEBUG_FS
struct lpfc_debugfs_trc {
	char *fmt;
	uint32_t data1;
	uint32_t data2;
	uint32_t data3;
	uint32_t seq_cnt;
	unsigned long jif;
};
#endif

/* Mask for discovery_trace */
#define LPFC_DISC_TRC_ELS_CMD		0x1	/* Trace ELS commands */
#define LPFC_DISC_TRC_ELS_RSP		0x2	/* Trace ELS response */
#define LPFC_DISC_TRC_ELS_UNSOL		0x4	/* Trace ELS rcv'ed   */
#define LPFC_DISC_TRC_ELS_ALL		0x7	/* Trace ELS */
#define LPFC_DISC_TRC_MBOX_VPORT	0x8	/* Trace vport MBOXs */
#define LPFC_DISC_TRC_MBOX		0x10	/* Trace other MBOXs */
#define LPFC_DISC_TRC_MBOX_ALL		0x18	/* Trace all MBOXs */
#define LPFC_DISC_TRC_CT		0x20	/* Trace disc CT requests */
#define LPFC_DISC_TRC_DSM		0x40    /* Trace DSM events */
#define LPFC_DISC_TRC_RPORT		0x80    /* Trace rport events */
#define LPFC_DISC_TRC_NODE		0x100   /* Trace ndlp state changes */

#define LPFC_DISC_TRC_DISCOVERY		0xef    /* common mask for general
						 * discovery */
#endif /* H_LPFC_DEBUG_FS */
