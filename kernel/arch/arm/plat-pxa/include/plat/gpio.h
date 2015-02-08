
#ifndef __PLAT_GPIO_H
#define __PLAT_GPIO_H


#define GPIO_BANK(n)	(GPIO_REGS_VIRT + BANK_OFF(n))

#define GPLR_OFFSET	0x00
#define GPDR_OFFSET	0x0C
#define GPSR_OFFSET	0x18
#define GPCR_OFFSET	0x24
#define GRER_OFFSET	0x30
#define GFER_OFFSET	0x3C
#define GEDR_OFFSET	0x48

static inline int gpio_get_value(unsigned gpio)
{
	if (__builtin_constant_p(gpio) && (gpio < NR_BUILTIN_GPIO))
		return GPLR(gpio) & GPIO_bit(gpio);
	else
		return __gpio_get_value(gpio);
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	if (__builtin_constant_p(gpio) && (gpio < NR_BUILTIN_GPIO)) {
		if (value)
			GPSR(gpio) = GPIO_bit(gpio);
		else
			GPCR(gpio) = GPIO_bit(gpio);
	} else
		__gpio_set_value(gpio, value);
}

#define gpio_cansleep		__gpio_cansleep

extern int pxa_last_gpio;

typedef int (*set_wake_t)(unsigned int irq, unsigned int on);

extern void pxa_init_gpio(int mux_irq, int start, int end, set_wake_t fn);
#endif /* __PLAT_GPIO_H */
