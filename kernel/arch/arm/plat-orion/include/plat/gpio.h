

#ifndef __PLAT_GPIO_H
#define __PLAT_GPIO_H

#include <linux/init.h>

#define gpio_get_value  __gpio_get_value
#define gpio_set_value  __gpio_set_value
#define gpio_cansleep   __gpio_cansleep

void orion_gpio_set_unused(unsigned pin);
void orion_gpio_set_blink(unsigned pin, int blink);

#define GPIO_INPUT_OK		(1 << 0)
#define GPIO_OUTPUT_OK		(1 << 1)
void orion_gpio_set_valid(unsigned pin, int mode);

/* Initialize gpiolib. */
void __init orion_gpio_init(void);

extern struct irq_chip orion_gpio_irq_chip;
void orion_gpio_irq_handler(int irqoff);


#endif
