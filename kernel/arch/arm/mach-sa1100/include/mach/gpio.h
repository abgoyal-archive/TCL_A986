

#ifndef __ASM_ARCH_SA1100_GPIO_H
#define __ASM_ARCH_SA1100_GPIO_H

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm-generic/gpio.h>

static inline int gpio_get_value(unsigned gpio)
{
	if (__builtin_constant_p(gpio) && (gpio <= GPIO_MAX))
		return GPLR & GPIO_GPIO(gpio);
	else
		return __gpio_get_value(gpio);
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	if (__builtin_constant_p(gpio) && (gpio <= GPIO_MAX))
		if (value)
			GPSR = GPIO_GPIO(gpio);
		else
			GPCR = GPIO_GPIO(gpio);
	else
		__gpio_set_value(gpio, value);
}

#define gpio_cansleep	__gpio_cansleep

#define gpio_to_irq(gpio)	((gpio < 11) ? (IRQ_GPIO0 + gpio) : \
					(IRQ_GPIO11 - 11 + gpio))
#define irq_to_gpio(irq)	((irq < IRQ_GPIO11_27) ? (irq - IRQ_GPIO0) : \
					(irq - IRQ_GPIO11 + 11))

#endif
