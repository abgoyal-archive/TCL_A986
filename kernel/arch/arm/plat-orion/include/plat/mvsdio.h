

#ifndef __MACH_MVSDIO_H
#define __MACH_MVSDIO_H

#include <linux/mbus.h>

struct mvsdio_platform_data {
	struct mbus_dram_target_info *dram;
	unsigned int clock;
	int gpio_card_detect;
	int gpio_write_protect;
};

#endif
