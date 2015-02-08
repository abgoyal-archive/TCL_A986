

#ifndef _LINUX_TIMED_GPIO_H
#define _LINUX_TIMED_GPIO_H

#define TIMED_GPIO_NAME "timed-gpio"

struct timed_gpio {
	const char *name;
	unsigned 	gpio;
	int		max_timeout;
	u8 		active_low;
};

struct timed_gpio_platform_data {
	int 		num_gpios;
	struct timed_gpio *gpios;
};

#endif
