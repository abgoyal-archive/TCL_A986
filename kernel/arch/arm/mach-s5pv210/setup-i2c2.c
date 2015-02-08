

#include <linux/kernel.h>
#include <linux/types.h>

struct platform_device; /* don't need the contents */

#include <mach/gpio.h>
#include <plat/iic.h>
#include <plat/gpio-cfg.h>

void s3c_i2c2_cfg_gpio(struct platform_device *dev)
{
	s3c_gpio_cfgpin(S5PV210_GPD1(4), S3C_GPIO_SFN(2));
	s3c_gpio_setpull(S5PV210_GPD1(4), S3C_GPIO_PULL_UP);
	s3c_gpio_cfgpin(S5PV210_GPD1(5), S3C_GPIO_SFN(2));
	s3c_gpio_setpull(S5PV210_GPD1(5), S3C_GPIO_PULL_UP);
}
