
#ifndef _LINUX_VIDEO_OUTPUT_H
#define _LINUX_VIDEO_OUTPUT_H
#include <linux/device.h>
struct output_device;
struct output_properties {
	int (*set_state)(struct output_device *);
	int (*get_status)(struct output_device *);
};
struct output_device {
	int request_state;
	struct output_properties *props;
	struct device dev;
};
#define to_output_device(obj) container_of(obj, struct output_device, dev)
struct output_device *video_output_register(const char *name,
	struct device *dev,
	void *devdata,
	struct output_properties *op);
void video_output_unregister(struct output_device *dev);
#endif
