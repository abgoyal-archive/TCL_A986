

#ifndef _VMW_PVSCSI_H_
#define _VMW_PVSCSI_H_

#include <linux/types.h>

#define PVSCSI_DRIVER_VERSION_STRING   "1.0.1.0-k"

#define PVSCSI_MAX_NUM_SG_ENTRIES_PER_SEGMENT 128

#define MASK(n)        ((1 << (n)) - 1)        /* make an n-bit mask */

#define PCI_VENDOR_ID_VMWARE		0x15AD
#define PCI_DEVICE_ID_VMWARE_PVSCSI	0x07C0

enum HostBusAdapterStatus {
   BTSTAT_SUCCESS       = 0x00,  /* CCB complete normally with no errors */
   BTSTAT_LINKED_COMMAND_COMPLETED           = 0x0a,
   BTSTAT_LINKED_COMMAND_COMPLETED_WITH_FLAG = 0x0b,
   BTSTAT_DATA_UNDERRUN = 0x0c,
   BTSTAT_SELTIMEO      = 0x11,  /* SCSI selection timeout */
   BTSTAT_DATARUN       = 0x12,  /* data overrun/underrun */
   BTSTAT_BUSFREE       = 0x13,  /* unexpected bus free */
   BTSTAT_INVPHASE      = 0x14,  /* invalid bus phase or sequence requested by target */
   BTSTAT_LUNMISMATCH   = 0x17,  /* linked CCB has different LUN from first CCB */
   BTSTAT_SENSFAILED    = 0x1b,  /* auto request sense failed */
   BTSTAT_TAGREJECT     = 0x1c,  /* SCSI II tagged queueing message rejected by target */
   BTSTAT_BADMSG        = 0x1d,  /* unsupported message received by the host adapter */
   BTSTAT_HAHARDWARE    = 0x20,  /* host adapter hardware failed */
   BTSTAT_NORESPONSE    = 0x21,  /* target did not respond to SCSI ATN, sent a SCSI RST */
   BTSTAT_SENTRST       = 0x22,  /* host adapter asserted a SCSI RST */
   BTSTAT_RECVRST       = 0x23,  /* other SCSI devices asserted a SCSI RST */
   BTSTAT_DISCONNECT    = 0x24,  /* target device reconnected improperly (w/o tag) */
   BTSTAT_BUSRESET      = 0x25,  /* host adapter issued BUS device reset */
   BTSTAT_ABORTQUEUE    = 0x26,  /* abort queue generated */
   BTSTAT_HASOFTWARE    = 0x27,  /* host adapter software error */
   BTSTAT_HATIMEOUT     = 0x30,  /* host adapter hardware timeout error */
   BTSTAT_SCSIPARITY    = 0x34,  /* SCSI parity error detected */
};


enum PVSCSIRegOffset {
	PVSCSI_REG_OFFSET_COMMAND        =    0x0,
	PVSCSI_REG_OFFSET_COMMAND_DATA   =    0x4,
	PVSCSI_REG_OFFSET_COMMAND_STATUS =    0x8,
	PVSCSI_REG_OFFSET_LAST_STS_0     =  0x100,
	PVSCSI_REG_OFFSET_LAST_STS_1     =  0x104,
	PVSCSI_REG_OFFSET_LAST_STS_2     =  0x108,
	PVSCSI_REG_OFFSET_LAST_STS_3     =  0x10c,
	PVSCSI_REG_OFFSET_INTR_STATUS    = 0x100c,
	PVSCSI_REG_OFFSET_INTR_MASK      = 0x2010,
	PVSCSI_REG_OFFSET_KICK_NON_RW_IO = 0x3014,
	PVSCSI_REG_OFFSET_DEBUG          = 0x3018,
	PVSCSI_REG_OFFSET_KICK_RW_IO     = 0x4018,
};


enum PVSCSICommands {
	PVSCSI_CMD_FIRST             = 0, /* has to be first */

	PVSCSI_CMD_ADAPTER_RESET     = 1,
	PVSCSI_CMD_ISSUE_SCSI        = 2,
	PVSCSI_CMD_SETUP_RINGS       = 3,
	PVSCSI_CMD_RESET_BUS         = 4,
	PVSCSI_CMD_RESET_DEVICE      = 5,
	PVSCSI_CMD_ABORT_CMD         = 6,
	PVSCSI_CMD_CONFIG            = 7,
	PVSCSI_CMD_SETUP_MSG_RING    = 8,
	PVSCSI_CMD_DEVICE_UNPLUG     = 9,

	PVSCSI_CMD_LAST              = 10  /* has to be last */
};


struct PVSCSICmdDescResetDevice {
	u32	target;
	u8	lun[8];
} __packed;


struct PVSCSICmdDescAbortCmd {
	u64	context;
	u32	target;
	u32	_pad;
} __packed;


#define PVSCSI_SETUP_RINGS_MAX_NUM_PAGES        32
struct PVSCSICmdDescSetupRings {
	u32	reqRingNumPages;
	u32	cmpRingNumPages;
	u64	ringsStatePPN;
	u64	reqRingPPNs[PVSCSI_SETUP_RINGS_MAX_NUM_PAGES];
	u64	cmpRingPPNs[PVSCSI_SETUP_RINGS_MAX_NUM_PAGES];
} __packed;


#define PVSCSI_SETUP_MSG_RING_MAX_NUM_PAGES  16

struct PVSCSICmdDescSetupMsgRing {
	u32	numPages;
	u32	_pad;
	u64	ringPPNs[PVSCSI_SETUP_MSG_RING_MAX_NUM_PAGES];
} __packed;

enum PVSCSIMsgType {
	PVSCSI_MSG_DEV_ADDED          = 0,
	PVSCSI_MSG_DEV_REMOVED        = 1,
	PVSCSI_MSG_LAST               = 2,
};


struct PVSCSIRingMsgDesc {
	u32	type;
	u32	args[31];
} __packed;

struct PVSCSIMsgDescDevStatusChanged {
	u32	type;  /* PVSCSI_MSG_DEV _ADDED / _REMOVED */
	u32	bus;
	u32	target;
	u8	lun[8];
	u32	pad[27];
} __packed;


struct PVSCSIRingsState {
	u32	reqProdIdx;
	u32	reqConsIdx;
	u32	reqNumEntriesLog2;

	u32	cmpProdIdx;
	u32	cmpConsIdx;
	u32	cmpNumEntriesLog2;

	u8	_pad[104];

	u32	msgProdIdx;
	u32	msgConsIdx;
	u32	msgNumEntriesLog2;
} __packed;


#define PVSCSI_FLAG_CMD_WITH_SG_LIST        (1 << 0)
#define PVSCSI_FLAG_CMD_OUT_OF_BAND_CDB     (1 << 1)
#define PVSCSI_FLAG_CMD_DIR_NONE            (1 << 2)
#define PVSCSI_FLAG_CMD_DIR_TOHOST          (1 << 3)
#define PVSCSI_FLAG_CMD_DIR_TODEVICE        (1 << 4)

struct PVSCSIRingReqDesc {
	u64	context;
	u64	dataAddr;
	u64	dataLen;
	u64	senseAddr;
	u32	senseLen;
	u32	flags;
	u8	cdb[16];
	u8	cdbLen;
	u8	lun[8];
	u8	tag;
	u8	bus;
	u8	target;
	u8	vcpuHint;
	u8	unused[59];
} __packed;


struct PVSCSISGElement {
	u64	addr;
	u32	length;
	u32	flags;
} __packed;


struct PVSCSIRingCmpDesc {
	u64	context;
	u64	dataLen;
	u32	senseLen;
	u16	hostStatus;
	u16	scsiStatus;
	u32	_pad[2];
} __packed;


#define PVSCSI_INTR_CMPL_0                 (1 << 0)
#define PVSCSI_INTR_CMPL_1                 (1 << 1)
#define PVSCSI_INTR_CMPL_MASK              MASK(2)

#define PVSCSI_INTR_MSG_0                  (1 << 2)
#define PVSCSI_INTR_MSG_1                  (1 << 3)
#define PVSCSI_INTR_MSG_MASK               (MASK(2) << 2)

#define PVSCSI_INTR_ALL_SUPPORTED          MASK(4)

#define PVSCSI_MAX_INTRS        24

#define PVSCSI_VECTOR_COMPLETION   0


#define PVSCSI_MAX_NUM_PAGES_REQ_RING   PVSCSI_SETUP_RINGS_MAX_NUM_PAGES
#define PVSCSI_MAX_NUM_PAGES_CMP_RING   PVSCSI_SETUP_RINGS_MAX_NUM_PAGES
#define PVSCSI_MAX_NUM_PAGES_MSG_RING   PVSCSI_SETUP_MSG_RING_MAX_NUM_PAGES

#define PVSCSI_MAX_NUM_REQ_ENTRIES_PER_PAGE \
				(PAGE_SIZE / sizeof(struct PVSCSIRingReqDesc))

#define PVSCSI_MAX_REQ_QUEUE_DEPTH \
	(PVSCSI_MAX_NUM_PAGES_REQ_RING * PVSCSI_MAX_NUM_REQ_ENTRIES_PER_PAGE)

#define PVSCSI_MEM_SPACE_COMMAND_NUM_PAGES     1
#define PVSCSI_MEM_SPACE_INTR_STATUS_NUM_PAGES 1
#define PVSCSI_MEM_SPACE_MISC_NUM_PAGES        2
#define PVSCSI_MEM_SPACE_KICK_IO_NUM_PAGES     2
#define PVSCSI_MEM_SPACE_MSIX_NUM_PAGES        2

enum PVSCSIMemSpace {
	PVSCSI_MEM_SPACE_COMMAND_PAGE		= 0,
	PVSCSI_MEM_SPACE_INTR_STATUS_PAGE	= 1,
	PVSCSI_MEM_SPACE_MISC_PAGE		= 2,
	PVSCSI_MEM_SPACE_KICK_IO_PAGE		= 4,
	PVSCSI_MEM_SPACE_MSIX_TABLE_PAGE	= 6,
	PVSCSI_MEM_SPACE_MSIX_PBA_PAGE		= 7,
};

#define PVSCSI_MEM_SPACE_NUM_PAGES \
	(PVSCSI_MEM_SPACE_COMMAND_NUM_PAGES +       \
	 PVSCSI_MEM_SPACE_INTR_STATUS_NUM_PAGES +   \
	 PVSCSI_MEM_SPACE_MISC_NUM_PAGES +          \
	 PVSCSI_MEM_SPACE_KICK_IO_NUM_PAGES +       \
	 PVSCSI_MEM_SPACE_MSIX_NUM_PAGES)

#define PVSCSI_MEM_SPACE_SIZE        (PVSCSI_MEM_SPACE_NUM_PAGES * PAGE_SIZE)

#endif /* _VMW_PVSCSI_H_ */
