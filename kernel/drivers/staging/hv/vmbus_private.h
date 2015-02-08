


#ifndef _VMBUS_PRIVATE_H_
#define _VMBUS_PRIVATE_H_

#include "hv.h"
#include "vmbus_api.h"
#include "channel.h"
#include "channel_mgmt.h"
#include "channel_interface.h"
#include "ring_buffer.h"
#include <linux/list.h>


#define MAX_NUM_CHANNELS	((PAGE_SIZE >> 1) << 3)	/* 16348 channels */

/* The value here must be in multiple of 32 */
/* TODO: Need to make this configurable */
#define MAX_NUM_CHANNELS_SUPPORTED	256


enum VMBUS_CONNECT_STATE {
	Disconnected,
	Connecting,
	Connected,
	Disconnecting
};

#define MAX_SIZE_CHANNEL_MESSAGE	HV_MESSAGE_PAYLOAD_BYTE_COUNT

struct VMBUS_CONNECTION {
	enum VMBUS_CONNECT_STATE ConnectState;

	atomic_t NextGpadlHandle;

	/*
	 * Represents channel interrupts. Each bit position represents a
	 * channel.  When a channel sends an interrupt via VMBUS, it finds its
	 * bit in the sendInterruptPage, set it and calls Hv to generate a port
	 * event. The other end receives the port event and parse the
	 * recvInterruptPage to see which bit is set
	 */
	void *InterruptPage;
	void *SendInterruptPage;
	void *RecvInterruptPage;

	/*
	 * 2 pages - 1st page for parent->child notification and 2nd
	 * is child->parent notification
	 */
	void *MonitorPages;
	struct list_head ChannelMsgList;
	spinlock_t channelmsg_lock;

	/* List of channels */
	struct list_head ChannelList;
	spinlock_t channel_lock;

	struct workqueue_struct *WorkQueue;
};


struct VMBUS_MSGINFO {
	/* Bookkeeping stuff */
	struct list_head MsgListEntry;

	/* Synchronize the request/response if needed */
	struct osd_waitevent *WaitEvent;

	/* The message itself */
	unsigned char Msg[0];
};


extern struct VMBUS_CONNECTION gVmbusConnection;

/* General vmbus interface */

struct hv_device *VmbusChildDeviceCreate(struct hv_guid *deviceType,
					 struct hv_guid *deviceInstance,
					 void *context);

int VmbusChildDeviceAdd(struct hv_device *Device);

void VmbusChildDeviceRemove(struct hv_device *Device);

/* static void */
/* VmbusChildDeviceDestroy( */
/* struct hv_device *); */

struct vmbus_channel *GetChannelFromRelId(u32 relId);


/* Connection interface */

int VmbusConnect(void);

int VmbusDisconnect(void);

int VmbusPostMessage(void *buffer, size_t bufSize);

int VmbusSetEvent(u32 childRelId);

void VmbusOnEvents(void);


#endif /* _VMBUS_PRIVATE_H_ */
