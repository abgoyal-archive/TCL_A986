

#ifndef __ASM_ARCH_GPIO_H
#define __ASM_ARCH_GPIO_H

#ifndef __ASSEMBLY__

#define GPIO_IN				0
#define GPIO_OUT			1

#define IXP2000_GPIO_LOW		0
#define IXP2000_GPIO_HIGH		1

extern void gpio_line_config(int line, int direction);

static inline int gpio_line_get(int line)
{
	return (((*IXP2000_GPIO_PLR) >> line) & 1);
}

static inline void gpio_line_set(int line, int value)
{
	if (value == IXP2000_GPIO_HIGH) {
		ixp2000_reg_write(IXP2000_GPIO_POSR, 1 << line);
	} else if (value == IXP2000_GPIO_LOW) {
		ixp2000_reg_write(IXP2000_GPIO_POCR, 1 << line);
	}
}

#endif /* !__ASSEMBLY__ */

#endif /* ASM_ARCH_IXP2000_GPIO_H_ */
