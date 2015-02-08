

/* TODO: Cleanup and shorten the namespace */

typedef struct {
	u8	bLength;
	u8	bDescriptorType;
	u8	bDescriptorSubtype;
	u16	bcdSpecVersion;
	u32	dwClockFrequency;
	u32	dwClockUpdateRes;
	u8	bCapabilities;
	u32	dwDeviceRegistersLocation;
	u32	dwHostMemoryRegion;
	u32	dwHostMemoryRegionSize;
	u32	dwHostHibernatMemRegion;
	u32	dwHostHibernatMemRegionSize;
} __attribute__((packed)) tmComResHWDescr_t;

typedef struct {
	u8	bLength;
	u8	bDescriptorType;
	u8	bDescriptorSubtype;
	u8	bFlags;
	u8	bInterfaceType;
	u8	bInterfaceId;
	u8	bBaseInterface;
	u8	bInterruptId;
	u8	bDebugInterruptId;
	u8	BARLocation;
	u8	Reserved[3];
} tmComResInterfaceDescr_t;

typedef struct {
	u64	CommandRing;
	u64	ResponseRing;
	u32	CommandWrite;
	u32	CommandRead;
	u32	ResponseWrite;
	u32	ResponseRead;
} tmComResBusDescr_t;

typedef enum {
	NONE		= 0,
	TYPE_BUS_PCI	= 1,
	TYPE_BUS_PCIe	= 2,
	TYPE_BUS_USB	= 3,
	TYPE_BUS_I2C	= 4
} tmBusType_t;

typedef struct {
	tmBusType_t Type;
	u16	m_wMaxReqSize;
	u8	*m_pdwSetRing;
	u32	m_dwSizeSetRing;
	u8	*m_pdwGetRing;
	u32	m_dwSizeGetRing;
	u32	*m_pdwSetWritePos;
	u32	*m_pdwSetReadPos;
	u32	*m_pdwGetWritePos;
	u32	*m_pdwGetReadPos;

	/* All access is protected */
	struct mutex lock;

} tmComResBusInfo_t;

typedef struct {
	u8	id;
	u8	flags;
	u16	size;
	u32	command;
	u16	controlselector;
	u8	seqno;
} __attribute__((packed)) tmComResInfo_t;

typedef enum {
	SET_CUR  = 0x01,
	GET_CUR  = 0x81,
	GET_MIN  = 0x82,
	GET_MAX  = 0x83,
	GET_RES  = 0x84,
	GET_LEN  = 0x85,
	GET_INFO = 0x86,
	GET_DEF  = 0x87
} tmComResCmd_t;

struct cmd {
	u8 seqno;
	u32 inuse;
	u32 timeout;
	u32 signalled;
	struct mutex lock;
	wait_queue_head_t wait;
};

typedef struct {
	u32	pathid;
	u32	size;
	void	*descriptor;
} tmDescriptor_t;

typedef struct {
	u8	len;
	u8	type;
	u8	subtype;
	u8	unitid;
} __attribute__((packed)) tmComResDescrHeader_t;

typedef struct {
	u8	len;
	u8	type;
	u8	subtype;
	u8	unitid;
	u32	devicetype;
	u16	deviceid;
	u32	numgpiopins;
	u8	numgpiogroups;
	u8	controlsize;
} __attribute__((packed)) tmComResExtDevDescrHeader_t;

typedef struct {
	u32	pin;
	u8	state;
} __attribute__((packed)) tmComResGPIO_t;

typedef struct {
	u8	len;
	u8	type;
	u8	subtype;
	u8	pathid;
} __attribute__((packed)) tmComResPathDescrHeader_t;

/* terminaltype */
typedef enum {
	ITT_ANTENNA              = 0x0203,
	LINE_CONNECTOR           = 0x0603,
	SPDIF_CONNECTOR          = 0x0605,
	COMPOSITE_CONNECTOR      = 0x0401,
	SVIDEO_CONNECTOR         = 0x0402,
	COMPONENT_CONNECTOR      = 0x0403,
	STANDARD_DMA             = 0xF101
} tmComResTermType_t;

typedef struct {
	u8	len;
	u8	type;
	u8	subtype;
	u8	terminalid;
	u16	terminaltype;
	u8	assocterminal;
	u8	iterminal;
	u8	controlsize;
} __attribute__((packed)) tmComResAntTermDescrHeader_t;

typedef struct {
	u8	len;
	u8	type;
	u8	subtype;
	u8	unitid;
	u8	sourceid;
	u8	iunit;
	u32	tuningstandards;
	u8	controlsize;
	u32	controls;
} __attribute__((packed)) tmComResTunerDescrHeader_t;

typedef enum {
	/* the buffer does not contain any valid data */
	TM_BUFFER_FLAG_EMPTY,

	/* the buffer is filled with valid data */
	TM_BUFFER_FLAG_DONE,

	/* the buffer is the dummy buffer - TODO??? */
	TM_BUFFER_FLAG_DUMMY_BUFFER
} tmBufferFlag_t;

typedef struct {
	u64		*pagetablevirt;
	u64		pagetablephys;
	u16		offset;
	u8		*context;
	u64		timestamp;
	tmBufferFlag_t	BufferFlag_t;
	u32		lostbuffers;
	u32		validbuffers;
	u64		*dummypagevirt;
	u64		dummypagephys;
	u64		*addressvirt;
} tmBuffer_t;

typedef struct {
	u32	bitspersample;
	u32	samplesperline;
	u32	numberoflines;
	u32	pitch;
	u32	linethreshold;
	u64	**pagetablelistvirt;
	u64	*pagetablelistphys;
	u32	numpagetables;
	u32	numpagetableentries;
} tmHWStreamParameters_t;

typedef struct {
	tmHWStreamParameters_t		HWStreamParameters_t;
	u64				qwDummyPageTablePhys;
	u64				*pDummyPageTableVirt;
} tmStreamParameters_t;

typedef struct {
	u8	len;
	u8	type;
	u8	subtyle;
	u8	unitid;
	u16	terminaltype;
	u8	assocterminal;
	u8	sourceid;
	u8	iterminal;
	u32	BARLocation;
	u8	flags;
	u8	interruptid;
	u8	buffercount;
	u8	metadatasize;
	u8	numformats;
	u8	controlsize;
} __attribute__((packed)) tmComResDMATermDescrHeader_t;

typedef struct {
	u8	len;
	u8	type;
	u8	subtype;
	u8	bFormatIndex;
	u8	bDataOffset;
	u8	bPacketLength;
	u8	bStrideLength;
	u8	guidStrideFormat[16];
} __attribute__((packed)) tmComResTSFormatDescrHeader_t;

