
#ifndef __LINUX_USB_WUSB_WA_H
#define __LINUX_USB_WUSB_WA_H

enum {
	WA_EXEC_RC_CMD = 40,	/* Radio Control command Request */
};

/* Wireless Adapter Requests ([WUSB] table 8-51) */
enum {
	WUSB_REQ_ADD_MMC_IE     = 20,
	WUSB_REQ_REMOVE_MMC_IE  = 21,
	WUSB_REQ_SET_NUM_DNTS   = 22,
	WUSB_REQ_SET_CLUSTER_ID = 23,
	WUSB_REQ_SET_DEV_INFO   = 24,
	WUSB_REQ_GET_TIME       = 25,
	WUSB_REQ_SET_STREAM_IDX = 26,
	WUSB_REQ_SET_WUSB_MAS   = 27,
	WUSB_REQ_CHAN_STOP      = 28,
};


/* Wireless Adapter WUSB Channel Time types ([WUSB] table 8-52) */
enum {
	WUSB_TIME_ADJ   = 0,
	WUSB_TIME_BPST  = 1,
	WUSB_TIME_WUSB  = 2,
};

enum {
	WA_ENABLE = 0x01,
	WA_RESET = 0x02,
	RPIPE_PAUSE = 0x1,
};

/* Responses from Get Status request ([WUSB] section 8.3.1.6) */
enum {
	WA_STATUS_ENABLED = 0x01,
	WA_STATUS_RESETTING = 0x02
};

enum rpipe_crs {
	RPIPE_CRS_CTL = 0x01,
	RPIPE_CRS_ISO = 0x02,
	RPIPE_CRS_BULK = 0x04,
	RPIPE_CRS_INTR = 0x08
};

struct usb_rpipe_descriptor {
	u8	bLength;
	u8	bDescriptorType;
	__le16  wRPipeIndex;
	__le16	wRequests;
	__le16	wBlocks;		/* rw if 0 */
	__le16	wMaxPacketSize;		/* rw? */
	u8	bHSHubAddress;		/* reserved: 0 */
	u8	bHSHubPort;		/* ??? FIXME ??? */
	u8	bSpeed;			/* rw: xfer rate 'enum uwb_phy_rate' */
	u8	bDeviceAddress;		/* rw: Target device address */
	u8	bEndpointAddress;	/* rw: Target EP address */
	u8	bDataSequence;		/* ro: Current Data sequence */
	__le32	dwCurrentWindow;	/* ro */
	u8	bMaxDataSequence;	/* ro?: max supported seq */
	u8	bInterval;		/* rw:  */
	u8	bOverTheAirInterval;	/* rw:  */
	u8	bmAttribute;		/* ro?  */
	u8	bmCharacteristics;	/* ro? enum rpipe_attr, supported xsactions */
	u8	bmRetryOptions;		/* rw? */
	__le16	wNumTransactionErrors;	/* rw */
} __attribute__ ((packed));

enum wa_notif_type {
	DWA_NOTIF_RWAKE = 0x91,
	DWA_NOTIF_PORTSTATUS = 0x92,
	WA_NOTIF_TRANSFER = 0x93,
	HWA_NOTIF_BPST_ADJ = 0x94,
	HWA_NOTIF_DN = 0x95,
};

struct wa_notif_hdr {
	u8 bLength;
	u8 bNotifyType;			/* enum wa_notif_type */
} __attribute__((packed));

struct hwa_notif_dn {
	struct wa_notif_hdr hdr;
	u8 bSourceDeviceAddr;		/* from errata 2005/07 */
	u8 bmAttributes;
	struct wusb_dn_hdr dndata[];
} __attribute__((packed));

/* [WUSB] section 8.3.3 */
enum wa_xfer_type {
	WA_XFER_TYPE_CTL = 0x80,
	WA_XFER_TYPE_BI = 0x81,		/* bulk/interrupt */
	WA_XFER_TYPE_ISO = 0x82,
	WA_XFER_RESULT = 0x83,
	WA_XFER_ABORT = 0x84,
};

/* [WUSB] section 8.3.3 */
struct wa_xfer_hdr {
	u8 bLength;			/* 0x18 */
	u8 bRequestType;		/* 0x80 WA_REQUEST_TYPE_CTL */
	__le16 wRPipe;			/* RPipe index */
	__le32 dwTransferID;		/* Host-assigned ID */
	__le32 dwTransferLength;	/* Length of data to xfer */
	u8 bTransferSegment;
} __attribute__((packed));

struct wa_xfer_ctl {
	struct wa_xfer_hdr hdr;
	u8 bmAttribute;
	__le16 wReserved;
	struct usb_ctrlrequest baSetupData;
} __attribute__((packed));

struct wa_xfer_bi {
	struct wa_xfer_hdr hdr;
	u8 bReserved;
	__le16 wReserved;
} __attribute__((packed));

struct wa_xfer_hwaiso {
	struct wa_xfer_hdr hdr;
	u8 bReserved;
	__le16 wPresentationTime;
	__le32 dwNumOfPackets;
	/* FIXME: u8 pktdata[]? */
} __attribute__((packed));

/* [WUSB] section 8.3.3.5 */
struct wa_xfer_abort {
	u8 bLength;
	u8 bRequestType;
	__le16 wRPipe;			/* RPipe index */
	__le32 dwTransferID;		/* Host-assigned ID */
} __attribute__((packed));

struct wa_notif_xfer {
	struct wa_notif_hdr hdr;
	u8 bEndpoint;
	u8 Reserved;
} __attribute__((packed));

/** Transfer result basic codes [WUSB] table 8-15 */
enum {
	WA_XFER_STATUS_SUCCESS,
	WA_XFER_STATUS_HALTED,
	WA_XFER_STATUS_DATA_BUFFER_ERROR,
	WA_XFER_STATUS_BABBLE,
	WA_XFER_RESERVED,
	WA_XFER_STATUS_NOT_FOUND,
	WA_XFER_STATUS_INSUFFICIENT_RESOURCE,
	WA_XFER_STATUS_TRANSACTION_ERROR,
	WA_XFER_STATUS_ABORTED,
	WA_XFER_STATUS_RPIPE_NOT_READY,
	WA_XFER_INVALID_FORMAT,
	WA_XFER_UNEXPECTED_SEGMENT_NUMBER,
	WA_XFER_STATUS_RPIPE_TYPE_MISMATCH,
};

/** [WUSB] section 8.3.3.4 */
struct wa_xfer_result {
	struct wa_notif_hdr hdr;
	__le32 dwTransferID;
	__le32 dwTransferLength;
	u8     bTransferSegment;
	u8     bTransferStatus;
	__le32 dwNumOfPackets;
} __attribute__((packed));

struct usb_wa_descriptor {
	u8	bLength;
	u8	bDescriptorType;
	u16	bcdWAVersion;
	u8	bNumPorts;		/* don't use!! */
	u8	bmAttributes;		/* Reserved == 0 */
	u16	wNumRPipes;
	u16	wRPipeMaxBlock;
	u8	bRPipeBlockSize;
	u8	bPwrOn2PwrGood;
	u8	bNumMMCIEs;
	u8	DeviceRemovable;	/* FIXME: in DWA this is up to 16 bytes */
} __attribute__((packed));

struct hwa_dev_info {
	u8	bmDeviceAvailability[32];       /* FIXME: ignored for now */
	u8	bDeviceAddress;
	__le16	wPHYRates;
	u8	bmDeviceAttribute;
} __attribute__((packed));

#endif /* #ifndef __LINUX_USB_WUSB_WA_H */
