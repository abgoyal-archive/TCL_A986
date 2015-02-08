

#ifndef	__STLS2F_GPIO_H
#define	__STLS2F_GPIO_H

#include <asm-generic/gpio.h>

extern void gpio_set_value(unsigned gpio, int value);
extern int gpio_get_value(unsigned gpio);
extern int gpio_cansleep(unsigned gpio);

static inline int gpio_to_irq(int gpio)
{
	return -EINVAL;
}

static inline int irq_to_gpio(int gpio)
{
	return -EINVAL;
}

#endif				/* __STLS2F_GPIO_H */
