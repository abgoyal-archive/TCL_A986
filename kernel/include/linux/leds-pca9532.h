

#ifndef __LINUX_PCA9532_H
#define __LINUX_PCA9532_H

#include <linux/leds.h>
#include <linux/workqueue.h>

enum pca9532_state {
	PCA9532_OFF  = 0x0,
	PCA9532_ON   = 0x1,
	PCA9532_PWM0 = 0x2,
	PCA9532_PWM1 = 0x3
};

enum pca9532_type { PCA9532_TYPE_NONE, PCA9532_TYPE_LED,
	PCA9532_TYPE_N2100_BEEP };

struct pca9532_led {
	u8 id;
	struct i2c_client *client;
	char *name;
	struct led_classdev ldev;
	struct work_struct work;
	enum pca9532_type type;
	enum pca9532_state state;
};

struct pca9532_platform_data {
	struct pca9532_led leds[16];
	u8 pwm[2];
	u8 psc[2];
};

#endif /* __LINUX_PCA9532_H */

