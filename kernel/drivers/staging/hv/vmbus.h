


#ifndef _VMBUS_H_
#define _VMBUS_H_

#include <linux/device.h>
#include "vmbus_api.h"

struct driver_context {
	struct hv_guid class_id;

	struct device_driver driver;

	/*
	 * Use these methods instead of the struct device_driver so 2.6 kernel
	 * stops complaining
	 * TODO - fix this!
	 */
	int (*probe)(struct device *);
	int (*remove)(struct device *);
	void (*shutdown)(struct device *);
};

struct vm_device {
	struct work_struct probe_failed_work_item;
	struct hv_guid class_id;
	struct hv_guid device_id;
	int probe_error;
	struct hv_device device_obj;
	struct device device;
};

static inline struct vm_device *to_vm_device(struct hv_device *d)
{
	return container_of(d, struct vm_device, device_obj);
}

static inline struct vm_device *device_to_vm_device(struct device *d)
{
	return container_of(d, struct vm_device, device);
}

static inline struct driver_context *driver_to_driver_context(struct device_driver *d)
{
	return container_of(d, struct driver_context, driver);
}


/* Vmbus interface */

int vmbus_child_driver_register(struct driver_context *driver_ctx);
void vmbus_child_driver_unregister(struct driver_context *driver_ctx);
void vmbus_get_interface(struct vmbus_channel_interface *interface);

extern struct completion hv_channel_ready;

#endif /* _VMBUS_H_ */
