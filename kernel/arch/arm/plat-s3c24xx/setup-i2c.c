

#include <linux/kernel.h>
#include <linux/gpio.h>

struct platform_device;

#include <plat/gpio-cfg.h>
#include <plat/iic.h>
#include <mach/hardware.h>
#include <mach/regs-gpio.h>

void s3c_i2c0_cfg_gpio(struct platform_device *dev)
{
	s3c_gpio_cfgpin(S3C2410_GPE(15), S3C2410_GPE15_IICSDA);
	s3c_gpio_cfgpin(S3C2410_GPE(14), S3C2410_GPE14_IICSCL);
}
