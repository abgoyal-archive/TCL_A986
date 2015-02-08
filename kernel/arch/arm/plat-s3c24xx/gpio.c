

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/gpio.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/gpio-fns.h>
#include <asm/irq.h>

#include <mach/regs-gpio.h>

#include <plat/gpio-core.h>

/* gpiolib wrappers until these are totally eliminated */

void s3c2410_gpio_pullup(unsigned int pin, unsigned int to)
{
	int ret;

	WARN_ON(to);	/* should be none of these left */

	if (!to) {
		/* if pull is enabled, try first with up, and if that
		 * fails, try using down */

		ret = s3c_gpio_setpull(pin, S3C_GPIO_PULL_UP);
		if (ret)
			s3c_gpio_setpull(pin, S3C_GPIO_PULL_DOWN);
	} else {
		s3c_gpio_setpull(pin, S3C_GPIO_PULL_NONE);
	}
}
EXPORT_SYMBOL(s3c2410_gpio_pullup);

void s3c2410_gpio_setpin(unsigned int pin, unsigned int to)
{
	/* do this via gpiolib until all users removed */

	gpio_request(pin, "temporary");
	gpio_set_value(pin, to);
	gpio_free(pin);
}

EXPORT_SYMBOL(s3c2410_gpio_setpin);

unsigned int s3c2410_gpio_getpin(unsigned int pin)
{
	struct s3c_gpio_chip *chip = s3c_gpiolib_getchip(pin);
	unsigned long offs = pin - chip->chip.base;

	return __raw_readl(chip->base + 0x04) & (1<< offs);
}

EXPORT_SYMBOL(s3c2410_gpio_getpin);

unsigned int s3c2410_modify_misccr(unsigned int clear, unsigned int change)
{
	unsigned long flags;
	unsigned long misccr;

	local_irq_save(flags);
	misccr = __raw_readl(S3C24XX_MISCCR);
	misccr &= ~clear;
	misccr ^= change;
	__raw_writel(misccr, S3C24XX_MISCCR);
	local_irq_restore(flags);

	return misccr;
}

EXPORT_SYMBOL(s3c2410_modify_misccr);
