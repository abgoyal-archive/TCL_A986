

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fb.h>

#include <mach/regs-fb.h>
#include <mach/gpio.h>
#include <plat/fb.h>
#include <plat/gpio-cfg.h>

extern void s3c64xx_fb_gpio_setup_24bpp(void)
{
	unsigned int gpio;

	for (gpio = S3C64XX_GPI(0); gpio <= S3C64XX_GPI(15); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}

	for (gpio = S3C64XX_GPJ(0); gpio <= S3C64XX_GPJ(11); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}
}
