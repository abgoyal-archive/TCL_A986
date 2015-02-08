
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/sysdev.h>

#include <mach/hardware.h>
#include <asm/mach/irq.h>

#include "generic.h"


static int GPIO_IRQ_rising_edge;
static int GPIO_IRQ_falling_edge;
static int GPIO_IRQ_mask = (1 << 11) - 1;

#define GPIO_11_27_IRQ(i)	((i) - 21)
#define GPIO11_27_MASK(irq)	(1 << GPIO_11_27_IRQ(irq))

static int sa1100_gpio_type(unsigned int irq, unsigned int type)
{
	unsigned int mask;

	if (irq <= 10)
		mask = 1 << irq;
	else
		mask = GPIO11_27_MASK(irq);

	if (type == IRQ_TYPE_PROBE) {
		if ((GPIO_IRQ_rising_edge | GPIO_IRQ_falling_edge) & mask)
			return 0;
		type = IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING;
	}

	if (type & IRQ_TYPE_EDGE_RISING) {
		GPIO_IRQ_rising_edge |= mask;
	} else
		GPIO_IRQ_rising_edge &= ~mask;
	if (type & IRQ_TYPE_EDGE_FALLING) {
		GPIO_IRQ_falling_edge |= mask;
	} else
		GPIO_IRQ_falling_edge &= ~mask;

	GRER = GPIO_IRQ_rising_edge & GPIO_IRQ_mask;
	GFER = GPIO_IRQ_falling_edge & GPIO_IRQ_mask;

	return 0;
}

static void sa1100_low_gpio_ack(unsigned int irq)
{
	GEDR = (1 << irq);
}

static void sa1100_low_gpio_mask(unsigned int irq)
{
	ICMR &= ~(1 << irq);
}

static void sa1100_low_gpio_unmask(unsigned int irq)
{
	ICMR |= 1 << irq;
}

static int sa1100_low_gpio_wake(unsigned int irq, unsigned int on)
{
	if (on)
		PWER |= 1 << irq;
	else
		PWER &= ~(1 << irq);
	return 0;
}

static struct irq_chip sa1100_low_gpio_chip = {
	.name		= "GPIO-l",
	.ack		= sa1100_low_gpio_ack,
	.mask		= sa1100_low_gpio_mask,
	.unmask		= sa1100_low_gpio_unmask,
	.set_type	= sa1100_gpio_type,
	.set_wake	= sa1100_low_gpio_wake,
};

static void
sa1100_high_gpio_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned int mask;

	mask = GEDR & 0xfffff800;
	do {
		/*
		 * clear down all currently active IRQ sources.
		 * We will be processing them all.
		 */
		GEDR = mask;

		irq = IRQ_GPIO11;
		mask >>= 11;
		do {
			if (mask & 1)
				generic_handle_irq(irq);
			mask >>= 1;
			irq++;
		} while (mask);

		mask = GEDR & 0xfffff800;
	} while (mask);
}

static void sa1100_high_gpio_ack(unsigned int irq)
{
	unsigned int mask = GPIO11_27_MASK(irq);

	GEDR = mask;
}

static void sa1100_high_gpio_mask(unsigned int irq)
{
	unsigned int mask = GPIO11_27_MASK(irq);

	GPIO_IRQ_mask &= ~mask;

	GRER &= ~mask;
	GFER &= ~mask;
}

static void sa1100_high_gpio_unmask(unsigned int irq)
{
	unsigned int mask = GPIO11_27_MASK(irq);

	GPIO_IRQ_mask |= mask;

	GRER = GPIO_IRQ_rising_edge & GPIO_IRQ_mask;
	GFER = GPIO_IRQ_falling_edge & GPIO_IRQ_mask;
}

static int sa1100_high_gpio_wake(unsigned int irq, unsigned int on)
{
	if (on)
		PWER |= GPIO11_27_MASK(irq);
	else
		PWER &= ~GPIO11_27_MASK(irq);
	return 0;
}

static struct irq_chip sa1100_high_gpio_chip = {
	.name		= "GPIO-h",
	.ack		= sa1100_high_gpio_ack,
	.mask		= sa1100_high_gpio_mask,
	.unmask		= sa1100_high_gpio_unmask,
	.set_type	= sa1100_gpio_type,
	.set_wake	= sa1100_high_gpio_wake,
};

static void sa1100_mask_irq(unsigned int irq)
{
	ICMR &= ~(1 << irq);
}

static void sa1100_unmask_irq(unsigned int irq)
{
	ICMR |= (1 << irq);
}

static int sa1100_set_wake(unsigned int irq, unsigned int on)
{
	if (irq == IRQ_RTCAlrm) {
		if (on)
			PWER |= PWER_RTC;
		else
			PWER &= ~PWER_RTC;
		return 0;
	}
	return -EINVAL;
}

static struct irq_chip sa1100_normal_chip = {
	.name		= "SC",
	.ack		= sa1100_mask_irq,
	.mask		= sa1100_mask_irq,
	.unmask		= sa1100_unmask_irq,
	.set_wake	= sa1100_set_wake,
};

static struct resource irq_resource = {
	.name	= "irqs",
	.start	= 0x90050000,
	.end	= 0x9005ffff,
};

static struct sa1100irq_state {
	unsigned int	saved;
	unsigned int	icmr;
	unsigned int	iclr;
	unsigned int	iccr;
} sa1100irq_state;

static int sa1100irq_suspend(struct sys_device *dev, pm_message_t state)
{
	struct sa1100irq_state *st = &sa1100irq_state;

	st->saved = 1;
	st->icmr = ICMR;
	st->iclr = ICLR;
	st->iccr = ICCR;

	/*
	 * Disable all GPIO-based interrupts.
	 */
	ICMR &= ~(IC_GPIO11_27|IC_GPIO10|IC_GPIO9|IC_GPIO8|IC_GPIO7|
		  IC_GPIO6|IC_GPIO5|IC_GPIO4|IC_GPIO3|IC_GPIO2|
		  IC_GPIO1|IC_GPIO0);

	/*
	 * Set the appropriate edges for wakeup.
	 */
	GRER = PWER & GPIO_IRQ_rising_edge;
	GFER = PWER & GPIO_IRQ_falling_edge;
	
	/*
	 * Clear any pending GPIO interrupts.
	 */
	GEDR = GEDR;

	return 0;
}

static int sa1100irq_resume(struct sys_device *dev)
{
	struct sa1100irq_state *st = &sa1100irq_state;

	if (st->saved) {
		ICCR = st->iccr;
		ICLR = st->iclr;

		GRER = GPIO_IRQ_rising_edge & GPIO_IRQ_mask;
		GFER = GPIO_IRQ_falling_edge & GPIO_IRQ_mask;

		ICMR = st->icmr;
	}
	return 0;
}

static struct sysdev_class sa1100irq_sysclass = {
	.name		= "sa11x0-irq",
	.suspend	= sa1100irq_suspend,
	.resume		= sa1100irq_resume,
};

static struct sys_device sa1100irq_device = {
	.id		= 0,
	.cls		= &sa1100irq_sysclass,
};

static int __init sa1100irq_init_devicefs(void)
{
	sysdev_class_register(&sa1100irq_sysclass);
	return sysdev_register(&sa1100irq_device);
}

device_initcall(sa1100irq_init_devicefs);

void __init sa1100_init_irq(void)
{
	unsigned int irq;

	request_resource(&iomem_resource, &irq_resource);

	/* disable all IRQs */
	ICMR = 0;

	/* all IRQs are IRQ, not FIQ */
	ICLR = 0;

	/* clear all GPIO edge detects */
	GFER = 0;
	GRER = 0;
	GEDR = -1;

	/*
	 * Whatever the doc says, this has to be set for the wait-on-irq
	 * instruction to work... on a SA1100 rev 9 at least.
	 */
	ICCR = 1;

	for (irq = 0; irq <= 10; irq++) {
		set_irq_chip(irq, &sa1100_low_gpio_chip);
		set_irq_handler(irq, handle_edge_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}

	for (irq = 12; irq <= 31; irq++) {
		set_irq_chip(irq, &sa1100_normal_chip);
		set_irq_handler(irq, handle_level_irq);
		set_irq_flags(irq, IRQF_VALID);
	}

	for (irq = 32; irq <= 48; irq++) {
		set_irq_chip(irq, &sa1100_high_gpio_chip);
		set_irq_handler(irq, handle_edge_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}

	/*
	 * Install handler for GPIO 11-27 edge detect interrupts
	 */
	set_irq_chip(IRQ_GPIO11_27, &sa1100_normal_chip);
	set_irq_chained_handler(IRQ_GPIO11_27, sa1100_high_gpio_handler);

	sa1100_init_gpio();
}
