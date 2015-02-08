

#include <linux/kernel.h>
#include <linux/types.h>

struct platform_device; /* don't need the contents */

#include <mach/gpio.h>
#include <mach/gpio-bank-b.h>
#include <plat/iic.h>
#include <plat/gpio-cfg.h>

void s3c_i2c1_cfg_gpio(struct platform_device *dev)
{
	s3c_gpio_cfgpin(S3C64XX_GPB(2), S3C64XX_GPB2_I2C_SCL1);
	s3c_gpio_cfgpin(S3C64XX_GPB(3), S3C64XX_GPB3_I2C_SDA1);
	s3c_gpio_setpull(S3C64XX_GPB(2), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C64XX_GPB(3), S3C_GPIO_PULL_UP);
}
