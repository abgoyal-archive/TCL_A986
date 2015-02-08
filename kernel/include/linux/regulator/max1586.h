

#ifndef REGULATOR_MAX1586
#define REGULATOR_MAX1586

#include <linux/regulator/machine.h>

#define MAX1586_V3 0
#define MAX1586_V6 1

/* precalculated values for v3_gain */
#define MAX1586_GAIN_NO_R24   1000000  /* 700000 .. 1475000 mV */
#define MAX1586_GAIN_R24_3k32 1051098  /* 735768 .. 1550369 mV */
#define MAX1586_GAIN_R24_5k11 1078648  /* 755053 .. 1591005 mV */
#define MAX1586_GAIN_R24_7k5  1115432  /* 780802 .. 1645262 mV */

struct max1586_subdev_data {
	int				id;
	char				*name;
	struct regulator_init_data	*platform_data;
};

struct max1586_platform_data {
	int num_subdevs;
	struct max1586_subdev_data *subdevs;
	int v3_gain;
};

#endif
