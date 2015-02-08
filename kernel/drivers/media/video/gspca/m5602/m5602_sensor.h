

#ifndef M5602_SENSOR_H_
#define M5602_SENSOR_H_

#include "m5602_bridge.h"

#define M5602_V4L2_CID_GREEN_BALANCE	(V4L2_CID_PRIVATE_BASE + 0)
#define M5602_V4L2_CID_NOISE_SUPPRESION	(V4L2_CID_PRIVATE_BASE + 1)

/* Enumerates all supported sensors */
enum sensors {
	OV9650_SENSOR	= 1,
	S5K83A_SENSOR	= 2,
	S5K4AA_SENSOR	= 3,
	MT9M111_SENSOR	= 4,
	PO1030_SENSOR	= 5,
	OV7660_SENSOR   = 6,
};

/* Enumerates all possible instruction types */
enum instruction {
	BRIDGE,
	SENSOR,
	SENSOR_LONG
};

struct m5602_sensor {
	/* Defines the name of a sensor */
	char name[32];

	/* What i2c address the sensor is connected to */
	u8 i2c_slave_id;

	/* Width of each i2c register (in bytes) */
	u8 i2c_regW;

	/* Probes if the sensor is connected */
	int (*probe)(struct sd *sd);

	/* Performs a initialization sequence */
	int (*init)(struct sd *sd);

	/* Executed when the camera starts to send data */
	int (*start)(struct sd *sd);

	/* Executed when the camera ends to send data */
	int (*stop)(struct sd *sd);

	/* Executed when the device is disconnected */
	void (*disconnect)(struct sd *sd);
};

#endif
