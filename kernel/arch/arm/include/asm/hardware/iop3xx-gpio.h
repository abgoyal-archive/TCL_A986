

#ifndef __ASM_ARM_HARDWARE_IOP3XX_GPIO_H
#define __ASM_ARM_HARDWARE_IOP3XX_GPIO_H

#include <mach/hardware.h>
#include <asm-generic/gpio.h>

#define IOP3XX_N_GPIOS	8

static inline int gpio_get_value(unsigned gpio)
{
	if (gpio > IOP3XX_N_GPIOS)
		return __gpio_get_value(gpio);

	return gpio_line_get(gpio);
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	if (gpio > IOP3XX_N_GPIOS) {
		__gpio_set_value(gpio, value);
		return;
	}
	gpio_line_set(gpio, value);
}

static inline int gpio_cansleep(unsigned gpio)
{
	if (gpio < IOP3XX_N_GPIOS)
		return 0;
	else
		return __gpio_cansleep(gpio);
}

static inline int gpio_to_irq(int gpio)
{
	return -EINVAL;
}

static inline int irq_to_gpio(int gpio)
{
	return -EINVAL;
}

#endif

