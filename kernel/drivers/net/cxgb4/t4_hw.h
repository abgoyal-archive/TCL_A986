

#ifndef __T4_HW_H
#define __T4_HW_H

#include <linux/types.h>

enum {
	NCHAN          = 4,     /* # of HW channels */
	MAX_MTU        = 9600,  /* max MAC MTU, excluding header + FCS */
	EEPROMSIZE     = 17408, /* Serial EEPROM physical size */
	EEPROMVSIZE    = 32768, /* Serial EEPROM virtual address space size */
	RSS_NENTRIES   = 2048,  /* # of entries in RSS mapping table */
	TCB_SIZE       = 128,   /* TCB size */
	NMTUS          = 16,    /* size of MTU table */
	NCCTRL_WIN     = 32,    /* # of congestion control windows */
	NEXACT_MAC     = 336,   /* # of exact MAC address filters */
	L2T_SIZE       = 4096,  /* # of L2T entries */
	MBOX_LEN       = 64,    /* mailbox size in bytes */
	TRACE_LEN      = 112,   /* length of trace data and mask */
	FILTER_OPT_LEN = 36,    /* filter tuple width for optional components */
	NWOL_PAT       = 8,     /* # of WoL patterns */
	WOL_PAT_LEN    = 128,   /* length of WoL patterns */
};

enum {
	SF_PAGE_SIZE = 256,           /* serial flash page size */
	SF_SEC_SIZE = 64 * 1024,      /* serial flash sector size */
	SF_SIZE = SF_SEC_SIZE * 16,   /* serial flash size */
};

enum { RSP_TYPE_FLBUF, RSP_TYPE_CPL, RSP_TYPE_INTR }; /* response entry types */

enum { MBOX_OWNER_NONE, MBOX_OWNER_FW, MBOX_OWNER_DRV };    /* mailbox owners */

enum {
	SGE_MAX_WR_LEN = 512,     /* max WR size in bytes */
	SGE_NTIMERS = 6,          /* # of interrupt holdoff timer values */
	SGE_NCOUNTERS = 4,        /* # of interrupt packet counter values */
};

struct sge_qstat {                /* data written to SGE queue status entries */
	__be32 qid;
	__be16 cidx;
	__be16 pidx;
};

struct rsp_ctrl {
	__be32 hdrbuflen_pidx;
	__be32 pldbuflen_qid;
	union {
		u8 type_gen;
		__be64 last_flit;
	};
};

#define RSPD_NEWBUF 0x80000000U
#define RSPD_LEN    0x7fffffffU

#define RSPD_GEN(x)  ((x) >> 7)
#define RSPD_TYPE(x) (((x) >> 4) & 3)

#define QINTR_CNT_EN       0x1
#define QINTR_TIMER_IDX(x) ((x) << 1)
#endif /* __T4_HW_H */
