
#ifndef __ASM_ARCH_GPIO_H
#define __ASM_ARCH_GPIO_H

#include <linux/kernel.h>
#include <linux/errno.h>

#define ARCH_NR_GPIOS 1024
#include <linux/sh_pfc.h>

#ifdef CONFIG_GPIOLIB

static inline int gpio_get_value(unsigned gpio)
{
	return __gpio_get_value(gpio);
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	__gpio_set_value(gpio, value);
}

static inline int gpio_cansleep(unsigned gpio)
{
	return __gpio_cansleep(gpio);
}

static inline int gpio_to_irq(unsigned gpio)
{
	return -ENOSYS;
}

static inline int irq_to_gpio(unsigned int irq)
{
	return -EINVAL;
}

#endif /* CONFIG_GPIOLIB */

#endif /* __ASM_ARCH_GPIO_H */
