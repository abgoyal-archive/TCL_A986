


#ifndef _CHANNEL_INTERFACE_H_
#define _CHANNEL_INTERFACE_H_

#include "vmbus_api.h"

void GetChannelInterface(struct vmbus_channel_interface *ChannelInterface);

void GetChannelInfo(struct hv_device *Device,
		    struct hv_device_info *DeviceInfo);

#endif /* _CHANNEL_INTERFACE_H_ */
