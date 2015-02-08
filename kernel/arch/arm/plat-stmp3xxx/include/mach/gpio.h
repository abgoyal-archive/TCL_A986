

#ifndef __ASM_PLAT_GPIO_H
#define __ASM_PLAT_GPIO_H

#define ARCH_NR_GPIOS	(32 * 3)
#define gpio_to_irq(gpio) __gpio_to_irq(gpio)
#define gpio_get_value(gpio) __gpio_get_value(gpio)
#define gpio_set_value(gpio, value) __gpio_set_value(gpio, value)

#include <asm-generic/gpio.h>

#endif /* __ASM_PLAT_GPIO_H */
