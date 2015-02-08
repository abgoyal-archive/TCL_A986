

#ifndef __LINUX_LEDS_REGULATOR_H
#define __LINUX_LEDS_REGULATOR_H


#include <linux/leds.h>

struct led_regulator_platform_data {
	char *name;                     /* LED name as expected by LED class */
	enum led_brightness brightness; /* initial brightness value */
};

#endif /* __LINUX_LEDS_REGULATOR_H */
