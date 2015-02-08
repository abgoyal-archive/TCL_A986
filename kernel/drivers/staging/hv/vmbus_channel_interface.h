

#ifndef __VMBUSCHANNELINTERFACE_H
#define __VMBUSCHANNELINTERFACE_H

#define VMBUS_REVISION_NUMBER		13

/* Make maximum size of pipe payload of 16K */
#define MAX_PIPE_DATA_PAYLOAD		(sizeof(u8) * 16384)

/* Define PipeMode values. */
#define VMBUS_PIPE_TYPE_BYTE		0x00000000
#define VMBUS_PIPE_TYPE_MESSAGE		0x00000004

/* The size of the user defined data buffer for non-pipe offers. */
#define MAX_USER_DEFINED_BYTES		120

/* The size of the user defined data buffer for pipe offers. */
#define MAX_PIPE_USER_DEFINED_BYTES	116

struct vmbus_channel_offer {
	struct hv_guid InterfaceType;
	struct hv_guid InterfaceInstance;
	u64 InterruptLatencyIn100nsUnits;
	u32 InterfaceRevision;
	u32 ServerContextAreaSize;	/* in bytes */
	u16 ChannelFlags;
	u16 MmioMegabytes;		/* in bytes * 1024 * 1024 */

	union {
		/* Non-pipes: The user has MAX_USER_DEFINED_BYTES bytes. */
		struct {
			unsigned char UserDefined[MAX_USER_DEFINED_BYTES];
		} Standard;

		/*
		 * Pipes:
		 * The following sructure is an integrated pipe protocol, which
		 * is implemented on top of standard user-defined data. Pipe
		 * clients have MAX_PIPE_USER_DEFINED_BYTES left for their own
		 * use.
		 */
		struct {
			u32  PipeMode;
			unsigned char UserDefined[MAX_PIPE_USER_DEFINED_BYTES];
		} Pipe;
	} u;
	u32 Padding;
} __attribute__((packed));

/* Server Flags */
#define VMBUS_CHANNEL_ENUMERATE_DEVICE_INTERFACE	1
#define VMBUS_CHANNEL_SERVER_SUPPORTS_TRANSFER_PAGES	2
#define VMBUS_CHANNEL_SERVER_SUPPORTS_GPADLS		4
#define VMBUS_CHANNEL_NAMED_PIPE_MODE			0x10
#define VMBUS_CHANNEL_LOOPBACK_OFFER			0x100
#define VMBUS_CHANNEL_PARENT_OFFER			0x200
#define VMBUS_CHANNEL_REQUEST_MONITORED_NOTIFICATION	0x400

#endif
