

#ifndef _VMBUSPACKETFORMAT_H_
#define _VMBUSPACKETFORMAT_H_

struct vmpacket_descriptor {
	u16 Type;
	u16 DataOffset8;
	u16 Length8;
	u16 Flags;
	u64 TransactionId;
} __attribute__((packed));

struct vmpacket_header {
	u32 PreviousPacketStartOffset;
	struct vmpacket_descriptor Descriptor;
} __attribute__((packed));

struct vmtransfer_page_range {
	u32 ByteCount;
	u32 ByteOffset;
} __attribute__((packed));

struct vmtransfer_page_packet_header {
	struct vmpacket_descriptor d;
	u16 TransferPageSetId;
	bool SenderOwnsSet;
	u8 Reserved;
	u32 RangeCount;
	struct vmtransfer_page_range Ranges[1];
} __attribute__((packed));

struct vmgpadl_packet_header {
	struct vmpacket_descriptor d;
	u32 Gpadl;
	u32 Reserved;
} __attribute__((packed));

struct vmadd_remove_transfer_page_set {
	struct vmpacket_descriptor d;
	u32 Gpadl;
	u16 TransferPageSetId;
	u16 Reserved;
} __attribute__((packed));

struct gpa_range {
	u32 ByteCount;
	u32 ByteOffset;
	u64 PfnArray[0];
};

struct vmestablish_gpadl {
	struct vmpacket_descriptor d;
	u32 Gpadl;
	u32 RangeCount;
	struct gpa_range Range[1];
} __attribute__((packed));

struct vmteardown_gpadl {
	struct vmpacket_descriptor d;
	u32 Gpadl;
	u32 Reserved;	/* for alignment to a 8-byte boundary */
} __attribute__((packed));

struct vmdata_gpa_direct {
	struct vmpacket_descriptor d;
	u32 Reserved;
	u32 RangeCount;
	struct gpa_range Range[1];
} __attribute__((packed));

/* This is the format for a Additional Data Packet. */
struct vmadditional_data {
	struct vmpacket_descriptor d;
	u64 TotalBytes;
	u32 ByteOffset;
	u32 ByteCount;
	unsigned char Data[1];
} __attribute__((packed));

union vmpacket_largest_possible_header {
	struct vmpacket_descriptor SimpleHeader;
	struct vmtransfer_page_packet_header TransferPageHeader;
	struct vmgpadl_packet_header GpadlHeader;
	struct vmadd_remove_transfer_page_set AddRemoveTransferPageHeader;
	struct vmestablish_gpadl EstablishGpadlHeader;
	struct vmteardown_gpadl TeardownGpadlHeader;
	struct vmdata_gpa_direct DataGpaDirectHeader;
};

#define VMPACKET_DATA_START_ADDRESS(__packet)	\
	(void *)(((unsigned char *)__packet) +	\
	 ((struct vmpacket_descriptor)__packet)->DataOffset8 * 8)

#define VMPACKET_DATA_LENGTH(__packet)		\
	((((struct vmpacket_descriptor)__packet)->Length8 -	\
	  ((struct vmpacket_descriptor)__packet)->DataOffset8) * 8)

#define VMPACKET_TRANSFER_MODE(__packet)	\
	(((struct IMPACT)__packet)->Type)

enum vmbus_packet_type {
	VmbusPacketTypeInvalid				= 0x0,
	VmbusPacketTypeSynch				= 0x1,
	VmbusPacketTypeAddTransferPageSet		= 0x2,
	VmbusPacketTypeRemoveTransferPageSet		= 0x3,
	VmbusPacketTypeEstablishGpadl			= 0x4,
	VmbusPacketTypeTearDownGpadl			= 0x5,
	VmbusPacketTypeDataInBand			= 0x6,
	VmbusPacketTypeDataUsingTransferPages		= 0x7,
	VmbusPacketTypeDataUsingGpadl			= 0x8,
	VmbusPacketTypeDataUsingGpaDirect		= 0x9,
	VmbusPacketTypeCancelRequest			= 0xa,
	VmbusPacketTypeCompletion			= 0xb,
	VmbusPacketTypeDataUsingAdditionalPackets	= 0xc,
	VmbusPacketTypeAdditionalData			= 0xd
};

#define VMBUS_DATA_PACKET_FLAG_COMPLETION_REQUESTED	1

#endif
