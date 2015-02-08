

#ifndef	__DAVINCI_GPIO_H
#define	__DAVINCI_GPIO_H

#include <linux/io.h>
#include <linux/spinlock.h>

#include <asm-generic/gpio.h>

#include <mach/irqs.h>
#include <mach/common.h>

#define DAVINCI_GPIO_BASE 0x01C67000

enum davinci_gpio_type {
	GPIO_TYPE_DAVINCI = 0,
};

#define	GPIO(X)		(X)		/* 0 <= X <= (DAVINCI_N_GPIO - 1) */

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio)	(16 * (bank) + (gpio))

struct davinci_gpio_controller {
	struct gpio_chip	chip;
	int			irq_base;
	spinlock_t		lock;
	void __iomem		*regs;
	void __iomem		*set_data;
	void __iomem		*clr_data;
	void __iomem		*in_data;
};

static inline struct davinci_gpio_controller *
__gpio_to_controller(unsigned gpio)
{
	struct davinci_gpio_controller *ctlrs = davinci_soc_info.gpio_ctlrs;
	int index = gpio / 32;

	if (!ctlrs || index >= davinci_soc_info.gpio_ctlrs_num)
		return NULL;

	return ctlrs + index;
}

static inline u32 __gpio_mask(unsigned gpio)
{
	return 1 << (gpio % 32);
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	if (__builtin_constant_p(value) && gpio < davinci_soc_info.gpio_num) {
		struct davinci_gpio_controller *ctlr;
		u32				mask;

		ctlr = __gpio_to_controller(gpio);
		mask = __gpio_mask(gpio);
		if (value)
			__raw_writel(mask, ctlr->set_data);
		else
			__raw_writel(mask, ctlr->clr_data);
		return;
	}

	__gpio_set_value(gpio, value);
}

static inline int gpio_get_value(unsigned gpio)
{
	struct davinci_gpio_controller *ctlr;

	if (!__builtin_constant_p(gpio) || gpio >= davinci_soc_info.gpio_num)
		return __gpio_get_value(gpio);

	ctlr = __gpio_to_controller(gpio);
	return __gpio_mask(gpio) & __raw_readl(ctlr->in_data);
}

static inline int gpio_cansleep(unsigned gpio)
{
	if (__builtin_constant_p(gpio) && gpio < davinci_soc_info.gpio_num)
		return 0;
	else
		return __gpio_cansleep(gpio);
}

static inline int gpio_to_irq(unsigned gpio)
{
	return __gpio_to_irq(gpio);
}

static inline int irq_to_gpio(unsigned irq)
{
	/* don't support the reverse mapping */
	return -ENOSYS;
}

#endif				/* __DAVINCI_GPIO_H */
