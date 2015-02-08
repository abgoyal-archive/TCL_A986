

#ifndef RADIO_SI4713_H
#define RADIO_SI4713_H

#include <linux/i2c.h>

#define SI4713_NAME "radio-si4713"

struct radio_si4713_platform_data {
	int i2c_bus;
	struct i2c_board_info *subdev_board_info;
};

#endif /* ifndef RADIO_SI4713_H*/
