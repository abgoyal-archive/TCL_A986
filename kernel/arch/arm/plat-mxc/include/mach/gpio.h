

#ifndef __ASM_ARCH_MXC_GPIO_H__
#define __ASM_ARCH_MXC_GPIO_H__

#include <linux/spinlock.h>
#include <mach/hardware.h>
#include <asm-generic/gpio.h>

/* use gpiolib dispatchers */
#define gpio_get_value		__gpio_get_value
#define gpio_set_value		__gpio_set_value
#define gpio_cansleep		__gpio_cansleep

#define gpio_to_irq(gpio)	(MXC_GPIO_IRQ_START + (gpio))
#define irq_to_gpio(irq)	((irq) - MXC_GPIO_IRQ_START)

struct mxc_gpio_port {
	void __iomem *base;
	int irq;
	int virtual_irq_start;
	struct gpio_chip chip;
	u32 both_edges;
	spinlock_t lock;
};

int mxc_gpio_init(struct mxc_gpio_port*, int);

#endif
