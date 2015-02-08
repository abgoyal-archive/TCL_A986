

#ifndef _TIMB_RADIO_
#define _TIMB_RADIO_ 1

#include <linux/i2c.h>

struct timb_radio_platform_data {
	int i2c_adapter; /* I2C adapter where the tuner and dsp are attached */
	struct {
		const char *module_name;
		struct i2c_board_info *info;
	} tuner;
	struct {
		const char *module_name;
		struct i2c_board_info *info;
	} dsp;
};

#endif
