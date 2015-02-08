

/* vstorage.w revision number.  This is used in the case of a version match, */
/* to alert the user that structure sizes may be mismatched even though the */
/* protocol versions match. */

#define REVISION_STRING(REVISION_) #REVISION_
#define FILL_VMSTOR_REVISION(RESULT_LVALUE_)				\
{									\
	char *revisionString = REVISION_STRING($Revision : 6 $) + 11;	\
	RESULT_LVALUE_ = 0;						\
	while (*revisionString >= '0' && *revisionString <= '9') {	\
		RESULT_LVALUE_ *= 10;					\
		RESULT_LVALUE_ += *revisionString - '0';		\
		revisionString++;					\
	}								\
}

/* Major/minor macros.  Minor version is in LSB, meaning that earlier flat */
/* version numbers will be interpreted as "0.x" (i.e., 1 becomes 0.1). */
#define VMSTOR_PROTOCOL_MAJOR(VERSION_)		(((VERSION_) >> 8) & 0xff)
#define VMSTOR_PROTOCOL_MINOR(VERSION_)		(((VERSION_))      & 0xff)
#define VMSTOR_PROTOCOL_VERSION(MAJOR_, MINOR_)	((((MAJOR_) & 0xff) << 8) | \
						 (((MINOR_) & 0xff)))
#define VMSTOR_INVALID_PROTOCOL_VERSION		(-1)

/* Version history: */
/* V1 Beta                    0.1 */
/* V1 RC < 2008/1/31          1.0 */
/* V1 RC > 2008/1/31          2.0 */
#define VMSTOR_PROTOCOL_VERSION_CURRENT VMSTOR_PROTOCOL_VERSION(2, 0)




/*  This will get replaced with the max transfer length that is possible on */
/*  the host adapter. */
/*  The max transfer length will be published when we offer a vmbus channel. */
#define MAX_TRANSFER_LENGTH	0x40000
#define DEFAULT_PACKET_SIZE (sizeof(struct vmdata_gpa_direct) +	\
			sizeof(struct vstor_packet) +		\
			sizesizeof(u64) * (MAX_TRANSFER_LENGTH / PAGE_SIZE)))


/*  Packet structure describing virtual storage requests. */
enum vstor_packet_operation {
	VStorOperationCompleteIo            = 1,
	VStorOperationRemoveDevice          = 2,
	VStorOperationExecuteSRB            = 3,
	VStorOperationResetLun              = 4,
	VStorOperationResetAdapter          = 5,
	VStorOperationResetBus              = 6,
	VStorOperationBeginInitialization   = 7,
	VStorOperationEndInitialization     = 8,
	VStorOperationQueryProtocolVersion  = 9,
	VStorOperationQueryProperties       = 10,
	VStorOperationMaximum               = 10
};

#define CDB16GENERIC_LENGTH			0x10

#ifndef SENSE_BUFFER_SIZE
#define SENSE_BUFFER_SIZE			0x12
#endif

#define MAX_DATA_BUFFER_LENGTH_WITH_PADDING	0x14

struct vmscsi_request {
	unsigned short Length;
	unsigned char SrbStatus;
	unsigned char ScsiStatus;

	unsigned char PortNumber;
	unsigned char PathId;
	unsigned char TargetId;
	unsigned char Lun;

	unsigned char CdbLength;
	unsigned char SenseInfoLength;
	unsigned char DataIn;
	unsigned char Reserved;

	unsigned int DataTransferLength;

	union {
	unsigned char Cdb[CDB16GENERIC_LENGTH];

	unsigned char SenseData[SENSE_BUFFER_SIZE];

	unsigned char ReservedArray[MAX_DATA_BUFFER_LENGTH_WITH_PADDING];
	};
} __attribute((packed));


struct vmstorage_channel_properties {
	unsigned short ProtocolVersion;
	unsigned char  PathId;
	unsigned char  TargetId;

	/* Note: port number is only really known on the client side */
	unsigned int  PortNumber;
	unsigned int  Flags;
	unsigned int  MaxTransferBytes;

	/*  This id is unique for each channel and will correspond with */
	/*  vendor specific data in the inquirydata */
	unsigned long long UniqueId;
} __attribute__((packed));

/*  This structure is sent during the storage protocol negotiations. */
struct vmstorage_protocol_version {
	/* Major (MSW) and minor (LSW) version numbers. */
	unsigned short MajorMinor;

	/*
	 * Revision number is auto-incremented whenever this file is changed
	 * (See FILL_VMSTOR_REVISION macro above).  Mismatch does not
	 * definitely indicate incompatibility--but it does indicate mismatched
	 * builds.
	 */
	unsigned short Revision;
} __attribute__((packed));

/* Channel Property Flags */
#define STORAGE_CHANNEL_REMOVABLE_FLAG		0x1
#define STORAGE_CHANNEL_EMULATED_IDE_FLAG	0x2

struct vstor_packet {
	/* Requested operation type */
	enum vstor_packet_operation Operation;

	/*  Flags - see below for values */
	unsigned int     Flags;

	/* Status of the request returned from the server side. */
	unsigned int     Status;

	/* Data payload area */
	union {
		/*
		 * Structure used to forward SCSI commands from the
		 * client to the server.
		 */
		struct vmscsi_request VmSrb;

		/* Structure used to query channel properties. */
		struct vmstorage_channel_properties StorageChannelProperties;

		/* Used during version negotiations. */
		struct vmstorage_protocol_version Version;
	};
} __attribute__((packed));

/* Packet flags */
#define REQUEST_COMPLETION_FLAG	0x1

/*  This is the set of flags that the vsc can set in any packets it sends */
#define VSC_LEGAL_FLAGS		(REQUEST_COMPLETION_FLAG)
