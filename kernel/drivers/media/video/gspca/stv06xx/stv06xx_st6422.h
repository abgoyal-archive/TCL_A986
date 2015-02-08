

#ifndef STV06XX_ST6422_H_
#define STV06XX_ST6422_H_

#include "stv06xx_sensor.h"

static int st6422_probe(struct sd *sd);
static int st6422_start(struct sd *sd);
static int st6422_init(struct sd *sd);
static int st6422_stop(struct sd *sd);
static void st6422_disconnect(struct sd *sd);

/* V4L2 controls supported by the driver */
static int st6422_get_brightness(struct gspca_dev *gspca_dev, __s32 *val);
static int st6422_set_brightness(struct gspca_dev *gspca_dev, __s32 val);
static int st6422_get_contrast(struct gspca_dev *gspca_dev, __s32 *val);
static int st6422_set_contrast(struct gspca_dev *gspca_dev, __s32 val);
static int st6422_get_gain(struct gspca_dev *gspca_dev, __s32 *val);
static int st6422_set_gain(struct gspca_dev *gspca_dev, __s32 val);
static int st6422_get_exposure(struct gspca_dev *gspca_dev, __s32 *val);
static int st6422_set_exposure(struct gspca_dev *gspca_dev, __s32 val);

const struct stv06xx_sensor stv06xx_sensor_st6422 = {
	.name = "ST6422",
	.init = st6422_init,
	.probe = st6422_probe,
	.start = st6422_start,
	.stop = st6422_stop,
	.disconnect = st6422_disconnect,
};

#endif
