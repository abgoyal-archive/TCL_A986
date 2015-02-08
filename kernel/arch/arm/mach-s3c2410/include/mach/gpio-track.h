

#ifndef __ASM_ARCH_GPIO_CORE_H
#define __ASM_ARCH_GPIO_CORE_H __FILE__

#include <mach/regs-gpio.h>

extern struct s3c_gpio_chip s3c24xx_gpios[];

static inline struct s3c_gpio_chip *s3c_gpiolib_getchip(unsigned int pin)
{
	struct s3c_gpio_chip *chip;

	if (pin > S3C_GPIO_END)
		return NULL;

	chip = &s3c24xx_gpios[pin/32];
	return ((pin - chip->chip.base) < chip->chip.ngpio) ? chip : NULL;
}

#endif /* __ASM_ARCH_GPIO_CORE_H */
