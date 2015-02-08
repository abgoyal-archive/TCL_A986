

#ifndef __SOC_CAMERA_H__
#define __SOC_CAMERA_H__

#include <linux/videodev2.h>
#include <media/soc_camera.h>

struct device;

struct soc_camera_platform_info {
	const char *format_name;
	unsigned long format_depth;
	struct v4l2_mbus_framefmt format;
	unsigned long bus_param;
	struct device *dev;
	int (*set_capture)(struct soc_camera_platform_info *info, int enable);
};

#endif /* __SOC_CAMERA_H__ */
