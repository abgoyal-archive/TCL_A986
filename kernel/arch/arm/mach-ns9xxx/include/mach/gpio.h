
#ifndef __ASM_ARCH_GPIO_H
#define __ASM_ARCH_GPIO_H

#include <asm/errno.h>

int gpio_request(unsigned gpio, const char *label);

void gpio_free(unsigned gpio);

int ns9xxx_gpio_configure(unsigned gpio, int inv, int func);

int gpio_direction_input(unsigned gpio);

int gpio_direction_output(unsigned gpio, int value);

int gpio_get_value(unsigned gpio);

void gpio_set_value(unsigned gpio, int value);

static inline int gpio_to_irq(unsigned gpio)
{
	return -EINVAL;
}

static inline int irq_to_gpio(unsigned irq)
{
	return -EINVAL;
}

/* get the cansleep() stubs */
#include <asm-generic/gpio.h>

#endif /* ifndef __ASM_ARCH_GPIO_H */
