

#ifndef _ALCHEMY_GPIO_AU1000_H_
#define _ALCHEMY_GPIO_AU1000_H_

#include <asm/mach-au1x00/au1000.h>

#define ALCHEMY_GPIO1_BASE	0
#define ALCHEMY_GPIO2_BASE	200

#define ALCHEMY_GPIO1_NUM	32
#define ALCHEMY_GPIO2_NUM	16
#define ALCHEMY_GPIO1_MAX 	(ALCHEMY_GPIO1_BASE + ALCHEMY_GPIO1_NUM - 1)
#define ALCHEMY_GPIO2_MAX	(ALCHEMY_GPIO2_BASE + ALCHEMY_GPIO2_NUM - 1)

#define MAKE_IRQ(intc, off)	(AU1000_INTC##intc##_INT_BASE + (off))


static inline int au1000_gpio1_to_irq(int gpio)
{
	return MAKE_IRQ(1, gpio - ALCHEMY_GPIO1_BASE);
}

static inline int au1000_gpio2_to_irq(int gpio)
{
	return -ENXIO;
}

static inline int au1000_irq_to_gpio(int irq)
{
	if ((irq >= AU1000_GPIO0_INT) && (irq <= AU1000_GPIO31_INT))
		return ALCHEMY_GPIO1_BASE + (irq - AU1000_GPIO0_INT) + 0;

	return -ENXIO;
}

static inline int au1500_gpio1_to_irq(int gpio)
{
	gpio -= ALCHEMY_GPIO1_BASE;

	switch (gpio) {
	case 0 ... 15:
	case 20:
	case 23 ... 28:	return MAKE_IRQ(1, gpio);
	}

	return -ENXIO;
}

static inline int au1500_gpio2_to_irq(int gpio)
{
	gpio -= ALCHEMY_GPIO2_BASE;

	switch (gpio) {
	case 0 ... 3:	return MAKE_IRQ(1, 16 + gpio - 0);
	case 4 ... 5:	return MAKE_IRQ(1, 21 + gpio - 4);
	case 6 ... 7:	return MAKE_IRQ(1, 29 + gpio - 6);
	}

	return -ENXIO;
}

static inline int au1500_irq_to_gpio(int irq)
{
	switch (irq) {
	case AU1500_GPIO0_INT ... AU1500_GPIO15_INT:
	case AU1500_GPIO20_INT:
	case AU1500_GPIO23_INT ... AU1500_GPIO28_INT:
		return ALCHEMY_GPIO1_BASE + (irq - AU1500_GPIO0_INT) + 0;
	case AU1500_GPIO200_INT ... AU1500_GPIO203_INT:
		return ALCHEMY_GPIO2_BASE + (irq - AU1500_GPIO200_INT) + 0;
	case AU1500_GPIO204_INT ... AU1500_GPIO205_INT:
		return ALCHEMY_GPIO2_BASE + (irq - AU1500_GPIO204_INT) + 4;
	case AU1500_GPIO206_INT ... AU1500_GPIO207_INT:
		return ALCHEMY_GPIO2_BASE + (irq - AU1500_GPIO206_INT) + 6;
	case AU1500_GPIO208_215_INT:
		return ALCHEMY_GPIO2_BASE + 8;
	}

	return -ENXIO;
}

static inline int au1100_gpio1_to_irq(int gpio)
{
	return MAKE_IRQ(1, gpio - ALCHEMY_GPIO1_BASE);
}

static inline int au1100_gpio2_to_irq(int gpio)
{
	gpio -= ALCHEMY_GPIO2_BASE;

	if ((gpio >= 8) && (gpio <= 15))
		return MAKE_IRQ(0, 29);		/* shared GPIO208_215 */

	return -ENXIO;
}

static inline int au1100_irq_to_gpio(int irq)
{
	switch (irq) {
	case AU1100_GPIO0_INT ... AU1100_GPIO31_INT:
		return ALCHEMY_GPIO1_BASE + (irq - AU1100_GPIO0_INT) + 0;
	case AU1100_GPIO208_215_INT:
		return ALCHEMY_GPIO2_BASE + 8;
	}

	return -ENXIO;
}

static inline int au1550_gpio1_to_irq(int gpio)
{
	gpio -= ALCHEMY_GPIO1_BASE;

	switch (gpio) {
	case 0 ... 15:
	case 20 ... 28:	return MAKE_IRQ(1, gpio);
	case 16 ... 17:	return MAKE_IRQ(1, 18 + gpio - 16);
	}

	return -ENXIO;
}

static inline int au1550_gpio2_to_irq(int gpio)
{
	gpio -= ALCHEMY_GPIO2_BASE;

	switch (gpio) {
	case 0:		return MAKE_IRQ(1, 16);
	case 1 ... 5:	return MAKE_IRQ(1, 17);	/* shared GPIO201_205 */
	case 6 ... 7:	return MAKE_IRQ(1, 29 + gpio - 6);
	case 8 ... 15:	return MAKE_IRQ(1, 31);	/* shared GPIO208_215 */
	}

	return -ENXIO;
}

static inline int au1550_irq_to_gpio(int irq)
{
	switch (irq) {
	case AU1550_GPIO0_INT ... AU1550_GPIO15_INT:
		return ALCHEMY_GPIO1_BASE + (irq - AU1550_GPIO0_INT) + 0;
	case AU1550_GPIO200_INT:
	case AU1550_GPIO201_205_INT:
		return ALCHEMY_GPIO2_BASE + (irq - AU1550_GPIO200_INT) + 0;
	case AU1550_GPIO16_INT ... AU1550_GPIO28_INT:
		return ALCHEMY_GPIO1_BASE + (irq - AU1550_GPIO16_INT) + 16;
	case AU1550_GPIO206_INT ... AU1550_GPIO208_215_INT:
		return ALCHEMY_GPIO2_BASE + (irq - AU1550_GPIO206_INT) + 6;
	}

	return -ENXIO;
}

static inline int au1200_gpio1_to_irq(int gpio)
{
	return MAKE_IRQ(1, gpio - ALCHEMY_GPIO1_BASE);
}

static inline int au1200_gpio2_to_irq(int gpio)
{
	gpio -= ALCHEMY_GPIO2_BASE;

	switch (gpio) {
	case 0 ... 2:	return MAKE_IRQ(0, 5 + gpio - 0);
	case 3:		return MAKE_IRQ(0, 22);
	case 4 ... 7:	return MAKE_IRQ(0, 24 + gpio - 4);
	case 8 ... 15:	return MAKE_IRQ(0, 28);	/* shared GPIO208_215 */
	}

	return -ENXIO;
}

static inline int au1200_irq_to_gpio(int irq)
{
	switch (irq) {
	case AU1200_GPIO0_INT ... AU1200_GPIO31_INT:
		return ALCHEMY_GPIO1_BASE + (irq - AU1200_GPIO0_INT) + 0;
	case AU1200_GPIO200_INT ... AU1200_GPIO202_INT:
		return ALCHEMY_GPIO2_BASE + (irq - AU1200_GPIO200_INT) + 0;
	case AU1200_GPIO203_INT:
		return ALCHEMY_GPIO2_BASE + 3;
	case AU1200_GPIO204_INT ... AU1200_GPIO208_215_INT:
		return ALCHEMY_GPIO2_BASE + (irq - AU1200_GPIO204_INT) + 4;
	}

	return -ENXIO;
}

static inline void alchemy_gpio1_set_value(int gpio, int v)
{
	unsigned long mask = 1 << (gpio - ALCHEMY_GPIO1_BASE);
	unsigned long r = v ? SYS_OUTPUTSET : SYS_OUTPUTCLR;
	au_writel(mask, r);
	au_sync();
}

static inline int alchemy_gpio1_get_value(int gpio)
{
	unsigned long mask = 1 << (gpio - ALCHEMY_GPIO1_BASE);
	return au_readl(SYS_PINSTATERD) & mask;
}

static inline int alchemy_gpio1_direction_input(int gpio)
{
	unsigned long mask = 1 << (gpio - ALCHEMY_GPIO1_BASE);
	au_writel(mask, SYS_TRIOUTCLR);
	au_sync();
	return 0;
}

static inline int alchemy_gpio1_direction_output(int gpio, int v)
{
	/* hardware switches to "output" mode when one of the two
	 * "set_value" registers is accessed.
	 */
	alchemy_gpio1_set_value(gpio, v);
	return 0;
}

static inline int alchemy_gpio1_is_valid(int gpio)
{
	return ((gpio >= ALCHEMY_GPIO1_BASE) && (gpio <= ALCHEMY_GPIO1_MAX));
}

static inline int alchemy_gpio1_to_irq(int gpio)
{
	switch (alchemy_get_cputype()) {
	case ALCHEMY_CPU_AU1000:
		return au1000_gpio1_to_irq(gpio);
	case ALCHEMY_CPU_AU1100:
		return au1100_gpio1_to_irq(gpio);
	case ALCHEMY_CPU_AU1500:
		return au1500_gpio1_to_irq(gpio);
	case ALCHEMY_CPU_AU1550:
		return au1550_gpio1_to_irq(gpio);
	case ALCHEMY_CPU_AU1200:
		return au1200_gpio1_to_irq(gpio);
	}
	return -ENXIO;
}

static inline void __alchemy_gpio2_mod_dir(int gpio, int to_out)
{
	unsigned long mask = 1 << (gpio - ALCHEMY_GPIO2_BASE);
	unsigned long d = au_readl(GPIO2_DIR);
	if (to_out)
		d |= mask;
	else
		d &= ~mask;
	au_writel(d, GPIO2_DIR);
	au_sync();
}

static inline void alchemy_gpio2_set_value(int gpio, int v)
{
	unsigned long mask;
	mask = ((v) ? 0x00010001 : 0x00010000) << (gpio - ALCHEMY_GPIO2_BASE);
	au_writel(mask, GPIO2_OUTPUT);
	au_sync();
}

static inline int alchemy_gpio2_get_value(int gpio)
{
	return au_readl(GPIO2_PINSTATE) & (1 << (gpio - ALCHEMY_GPIO2_BASE));
}

static inline int alchemy_gpio2_direction_input(int gpio)
{
	unsigned long flags;
	local_irq_save(flags);
	__alchemy_gpio2_mod_dir(gpio, 0);
	local_irq_restore(flags);
	return 0;
}

static inline int alchemy_gpio2_direction_output(int gpio, int v)
{
	unsigned long flags;
	alchemy_gpio2_set_value(gpio, v);
	local_irq_save(flags);
	__alchemy_gpio2_mod_dir(gpio, 1);
	local_irq_restore(flags);
	return 0;
}

static inline int alchemy_gpio2_is_valid(int gpio)
{
	return ((gpio >= ALCHEMY_GPIO2_BASE) && (gpio <= ALCHEMY_GPIO2_MAX));
}

static inline int alchemy_gpio2_to_irq(int gpio)
{
	switch (alchemy_get_cputype()) {
	case ALCHEMY_CPU_AU1000:
		return au1000_gpio2_to_irq(gpio);
	case ALCHEMY_CPU_AU1100:
		return au1100_gpio2_to_irq(gpio);
	case ALCHEMY_CPU_AU1500:
		return au1500_gpio2_to_irq(gpio);
	case ALCHEMY_CPU_AU1550:
		return au1550_gpio2_to_irq(gpio);
	case ALCHEMY_CPU_AU1200:
		return au1200_gpio2_to_irq(gpio);
	}
	return -ENXIO;
}

/**********************************************************************/

static inline void alchemy_gpio1_input_enable(void)
{
	au_writel(0, SYS_PININPUTEN);	/* the write op is key */
	au_sync();
}

/* GPIO2 shared interrupts and control */

static inline void __alchemy_gpio2_mod_int(int gpio2, int en)
{
	unsigned long r = au_readl(GPIO2_INTENABLE);
	if (en)
		r |= 1 << gpio2;
	else
		r &= ~(1 << gpio2);
	au_writel(r, GPIO2_INTENABLE);
	au_sync();
}

static inline void alchemy_gpio2_enable_int(int gpio2)
{
	unsigned long flags;

	gpio2 -= ALCHEMY_GPIO2_BASE;

	/* Au1100/Au1500 have GPIO208-215 enable bits at 0..7 */
	switch (alchemy_get_cputype()) {
	case ALCHEMY_CPU_AU1100:
	case ALCHEMY_CPU_AU1500:
		gpio2 -= 8;
	}

	local_irq_save(flags);
	__alchemy_gpio2_mod_int(gpio2, 1);
	local_irq_restore(flags);
}

static inline void alchemy_gpio2_disable_int(int gpio2)
{
	unsigned long flags;

	gpio2 -= ALCHEMY_GPIO2_BASE;

	/* Au1100/Au1500 have GPIO208-215 enable bits at 0..7 */
	switch (alchemy_get_cputype()) {
	case ALCHEMY_CPU_AU1100:
	case ALCHEMY_CPU_AU1500:
		gpio2 -= 8;
	}

	local_irq_save(flags);
	__alchemy_gpio2_mod_int(gpio2, 0);
	local_irq_restore(flags);
}

static inline void alchemy_gpio2_enable(void)
{
	au_writel(3, GPIO2_ENABLE);	/* reset, clock enabled */
	au_sync();
	au_writel(1, GPIO2_ENABLE);	/* clock enabled */
	au_sync();
}

static inline void alchemy_gpio2_disable(void)
{
	au_writel(2, GPIO2_ENABLE);	/* reset, clock disabled */
	au_sync();
}

/**********************************************************************/

static inline int alchemy_gpio_direction_input(int gpio)
{
	return (gpio >= ALCHEMY_GPIO2_BASE) ?
		alchemy_gpio2_direction_input(gpio) :
		alchemy_gpio1_direction_input(gpio);
}

static inline int alchemy_gpio_direction_output(int gpio, int v)
{
	return (gpio >= ALCHEMY_GPIO2_BASE) ?
		alchemy_gpio2_direction_output(gpio, v) :
		alchemy_gpio1_direction_output(gpio, v);
}

static inline int alchemy_gpio_get_value(int gpio)
{
	return (gpio >= ALCHEMY_GPIO2_BASE) ?
		alchemy_gpio2_get_value(gpio) :
		alchemy_gpio1_get_value(gpio);
}

static inline void alchemy_gpio_set_value(int gpio, int v)
{
	if (gpio >= ALCHEMY_GPIO2_BASE)
		alchemy_gpio2_set_value(gpio, v);
	else
		alchemy_gpio1_set_value(gpio, v);
}

static inline int alchemy_gpio_is_valid(int gpio)
{
	return (gpio >= ALCHEMY_GPIO2_BASE) ?
		alchemy_gpio2_is_valid(gpio) :
		alchemy_gpio1_is_valid(gpio);
}

static inline int alchemy_gpio_cansleep(int gpio)
{
	return 0;	/* Alchemy never gets tired */
}

static inline int alchemy_gpio_to_irq(int gpio)
{
	return (gpio >= ALCHEMY_GPIO2_BASE) ?
		alchemy_gpio2_to_irq(gpio) :
		alchemy_gpio1_to_irq(gpio);
}

static inline int alchemy_irq_to_gpio(int irq)
{
	switch (alchemy_get_cputype()) {
	case ALCHEMY_CPU_AU1000:
		return au1000_irq_to_gpio(irq);
	case ALCHEMY_CPU_AU1100:
		return au1100_irq_to_gpio(irq);
	case ALCHEMY_CPU_AU1500:
		return au1500_irq_to_gpio(irq);
	case ALCHEMY_CPU_AU1550:
		return au1550_irq_to_gpio(irq);
	case ALCHEMY_CPU_AU1200:
		return au1200_irq_to_gpio(irq);
	}
	return -ENXIO;
}

/**********************************************************************/


#ifndef CONFIG_GPIOLIB


#ifndef CONFIG_ALCHEMY_GPIO_INDIRECT	/* case (4) */

static inline int gpio_direction_input(int gpio)
{
	return alchemy_gpio_direction_input(gpio);
}

static inline int gpio_direction_output(int gpio, int v)
{
	return alchemy_gpio_direction_output(gpio, v);
}

static inline int gpio_get_value(int gpio)
{
	return alchemy_gpio_get_value(gpio);
}

static inline void gpio_set_value(int gpio, int v)
{
	alchemy_gpio_set_value(gpio, v);
}

static inline int gpio_is_valid(int gpio)
{
	return alchemy_gpio_is_valid(gpio);
}

static inline int gpio_cansleep(int gpio)
{
	return alchemy_gpio_cansleep(gpio);
}

static inline int gpio_to_irq(int gpio)
{
	return alchemy_gpio_to_irq(gpio);
}

static inline int irq_to_gpio(int irq)
{
	return alchemy_irq_to_gpio(irq);
}

static inline int gpio_request(unsigned gpio, const char *label)
{
	return 0;
}

static inline void gpio_free(unsigned gpio)
{
}

#endif	/* !CONFIG_ALCHEMY_GPIO_INDIRECT */


#else	/* CONFIG GPIOLIB */


 /* using gpiolib to provide up to 2 gpio_chips for on-chip gpios */
#ifndef CONFIG_ALCHEMY_GPIO_INDIRECT	/* case (2) */

/* get everything through gpiolib */
#define gpio_to_irq	__gpio_to_irq
#define gpio_get_value	__gpio_get_value
#define gpio_set_value	__gpio_set_value
#define gpio_cansleep	__gpio_cansleep
#define irq_to_gpio	alchemy_irq_to_gpio

#include <asm-generic/gpio.h>

#endif	/* !CONFIG_ALCHEMY_GPIO_INDIRECT */


#endif	/* !CONFIG_GPIOLIB */

#endif /* _ALCHEMY_GPIO_AU1000_H_ */
