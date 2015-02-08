

#ifndef _HWMON_H_
#define _HWMON_H_

#include <linux/device.h>

struct device *hwmon_device_register(struct device *dev);

void hwmon_device_unregister(struct device *dev);

/* Scale user input to sensible values */
static inline int SENSORS_LIMIT(long value, long low, long high)
{
	if (value < low)
		return low;
	else if (value > high)
		return high;
	else
		return value;
}

#endif

