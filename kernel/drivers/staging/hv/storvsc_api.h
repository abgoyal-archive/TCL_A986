


#ifndef _STORVSC_API_H_
#define _STORVSC_API_H_

#include "vmbus_api.h"

/* Defines */
#define STORVSC_RING_BUFFER_SIZE			(20*PAGE_SIZE)
#define BLKVSC_RING_BUFFER_SIZE				(20*PAGE_SIZE)

#define STORVSC_MAX_IO_REQUESTS				128

#define STORVSC_MAX_LUNS_PER_TARGET			64
#define STORVSC_MAX_TARGETS				1
#define STORVSC_MAX_CHANNELS				1

struct hv_storvsc_request;

/* Matches Windows-end */
enum storvsc_request_type{
	WRITE_TYPE,
	READ_TYPE,
	UNKNOWN_TYPE,
};

struct hv_storvsc_request {
	enum storvsc_request_type Type;
	u32 Host;
	u32 Bus;
	u32 TargetId;
	u32 LunId;
	u8 *Cdb;
	u32 CdbLen;
	u32 Status;
	u32 BytesXfer;

	unsigned char *SenseBuffer;
	u32 SenseBufferSize;

	void *Context;

	void (*OnIOCompletion)(struct hv_storvsc_request *Request);

	/* This points to the memory after DataBuffer */
	void *Extension;

	struct hv_multipage_buffer DataBuffer;
};

/* Represents the block vsc driver */
struct storvsc_driver_object {
	/* Must be the first field */
	/* Which is a bug FIXME! */
	struct hv_driver Base;

	/* Set by caller (in bytes) */
	u32 RingBufferSize;

	/* Allocate this much private extension for each I/O request */
	u32 RequestExtSize;

	/* Maximum # of requests in flight per channel/device */
	u32 MaxOutstandingRequestsPerChannel;

	/* Specific to this driver */
	int (*OnIORequest)(struct hv_device *Device,
			   struct hv_storvsc_request *Request);
};

struct storvsc_device_info {
	unsigned int PortNumber;
	unsigned char PathId;
	unsigned char TargetId;
};

/* Interface */
int StorVscInitialize(struct hv_driver *driver);
int StorVscOnHostReset(struct hv_device *Device);
int BlkVscInitialize(struct hv_driver *driver);

#endif /* _STORVSC_API_H_ */
