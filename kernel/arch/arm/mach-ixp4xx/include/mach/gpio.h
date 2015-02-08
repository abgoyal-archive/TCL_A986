

#ifndef __ASM_ARCH_IXP4XX_GPIO_H
#define __ASM_ARCH_IXP4XX_GPIO_H

#include <linux/kernel.h>
#include <mach/hardware.h>

static inline int gpio_request(unsigned gpio, const char *label)
{
	return 0;
}

static inline void gpio_free(unsigned gpio)
{
	might_sleep();

	return;
}

static inline int gpio_direction_input(unsigned gpio)
{
	gpio_line_config(gpio, IXP4XX_GPIO_IN);
	return 0;
}

static inline int gpio_direction_output(unsigned gpio, int level)
{
	gpio_line_set(gpio, level);
	gpio_line_config(gpio, IXP4XX_GPIO_OUT);
	return 0;
}

static inline int gpio_get_value(unsigned gpio)
{
	int value;

	gpio_line_get(gpio, &value);

	return value;
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	gpio_line_set(gpio, value);
}

#include <asm-generic/gpio.h>			/* cansleep wrappers */

extern int gpio_to_irq(int gpio);
extern int irq_to_gpio(unsigned int irq);

#endif

