

#ifndef __LINUX_OF_GPIO_H
#define __LINUX_OF_GPIO_H

#include <linux/compiler.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/gpio.h>

struct device_node;

enum of_gpio_flags {
	OF_GPIO_ACTIVE_LOW = 0x1,
};

#ifdef CONFIG_OF_GPIO

struct of_gpio_chip {
	struct gpio_chip gc;
	int gpio_cells;
	int (*xlate)(struct of_gpio_chip *of_gc, struct device_node *np,
		     const void *gpio_spec, enum of_gpio_flags *flags);
};

static inline struct of_gpio_chip *to_of_gpio_chip(struct gpio_chip *gc)
{
	return container_of(gc, struct of_gpio_chip, gc);
}

struct of_mm_gpio_chip {
	struct of_gpio_chip of_gc;
	void (*save_regs)(struct of_mm_gpio_chip *mm_gc);
	void __iomem *regs;
};

static inline struct of_mm_gpio_chip *to_of_mm_gpio_chip(struct gpio_chip *gc)
{
	struct of_gpio_chip *of_gc = to_of_gpio_chip(gc);

	return container_of(of_gc, struct of_mm_gpio_chip, of_gc);
}

extern int of_get_gpio_flags(struct device_node *np, int index,
			     enum of_gpio_flags *flags);
extern unsigned int of_gpio_count(struct device_node *np);

extern int of_mm_gpiochip_add(struct device_node *np,
			      struct of_mm_gpio_chip *mm_gc);
extern int of_gpio_simple_xlate(struct of_gpio_chip *of_gc,
				struct device_node *np,
				const void *gpio_spec,
				enum of_gpio_flags *flags);
#else

/* Drivers may not strictly depend on the GPIO support, so let them link. */
static inline int of_get_gpio_flags(struct device_node *np, int index,
				    enum of_gpio_flags *flags)
{
	return -ENOSYS;
}

static inline unsigned int of_gpio_count(struct device_node *np)
{
	return 0;
}

#endif /* CONFIG_OF_GPIO */

static inline int of_get_gpio(struct device_node *np, int index)
{
	return of_get_gpio_flags(np, index, NULL);
}

#endif /* __LINUX_OF_GPIO_H */
