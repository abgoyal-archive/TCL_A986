

#ifndef __AR7_GPIO_H__
#define __AR7_GPIO_H__

#include <asm/mach-ar7/ar7.h>

#define AR7_GPIO_MAX 32
#define NR_BUILTIN_GPIO AR7_GPIO_MAX

#define gpio_to_irq(gpio)	-1

#define gpio_get_value __gpio_get_value
#define gpio_set_value __gpio_set_value

#define gpio_cansleep __gpio_cansleep

/* Board specific GPIO functions */
int ar7_gpio_enable(unsigned gpio);
int ar7_gpio_disable(unsigned gpio);

#include <asm-generic/gpio.h>

#endif
