
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>

#include <asm/mach/irq.h>
#include <mach/hardware.h>
#include <asm/irq.h>

#include <asm/hardware/clps7111.h>

static void int1_mask(unsigned int irq)
{
	u32 intmr1;

	intmr1 = clps_readl(INTMR1);
	intmr1 &= ~(1 << irq);
	clps_writel(intmr1, INTMR1);
}

static void int1_ack(unsigned int irq)
{
	u32 intmr1;

	intmr1 = clps_readl(INTMR1);
	intmr1 &= ~(1 << irq);
	clps_writel(intmr1, INTMR1);

	switch (irq) {
	case IRQ_CSINT:  clps_writel(0, COEOI);  break;
	case IRQ_TC1OI:  clps_writel(0, TC1EOI); break;
	case IRQ_TC2OI:  clps_writel(0, TC2EOI); break;
	case IRQ_RTCMI:  clps_writel(0, RTCEOI); break;
	case IRQ_TINT:   clps_writel(0, TEOI);   break;
	case IRQ_UMSINT: clps_writel(0, UMSEOI); break;
	}
}

static void int1_unmask(unsigned int irq)
{
	u32 intmr1;

	intmr1 = clps_readl(INTMR1);
	intmr1 |= 1 << irq;
	clps_writel(intmr1, INTMR1);
}

static struct irq_chip int1_chip = {
	.ack	= int1_ack,
	.mask	= int1_mask,
	.unmask = int1_unmask,
};

static void int2_mask(unsigned int irq)
{
	u32 intmr2;

	intmr2 = clps_readl(INTMR2);
	intmr2 &= ~(1 << (irq - 16));
	clps_writel(intmr2, INTMR2);
}

static void int2_ack(unsigned int irq)
{
	u32 intmr2;

	intmr2 = clps_readl(INTMR2);
	intmr2 &= ~(1 << (irq - 16));
	clps_writel(intmr2, INTMR2);

	switch (irq) {
	case IRQ_KBDINT: clps_writel(0, KBDEOI); break;
	}
}

static void int2_unmask(unsigned int irq)
{
	u32 intmr2;

	intmr2 = clps_readl(INTMR2);
	intmr2 |= 1 << (irq - 16);
	clps_writel(intmr2, INTMR2);
}

static struct irq_chip int2_chip = {
	.ack	= int2_ack,
	.mask	= int2_mask,
	.unmask = int2_unmask,
};

void __init clps711x_init_irq(void)
{
	unsigned int i;

	for (i = 0; i < NR_IRQS; i++) {
	        if (INT1_IRQS & (1 << i)) {
	        	set_irq_handler(i, handle_level_irq);
	        	set_irq_chip(i, &int1_chip);
	        	set_irq_flags(i, IRQF_VALID | IRQF_PROBE);
		}
		if (INT2_IRQS & (1 << i)) {
			set_irq_handler(i, handle_level_irq);
			set_irq_chip(i, &int2_chip);
			set_irq_flags(i, IRQF_VALID | IRQF_PROBE);
		}			
	}

	/*
	 * Disable interrupts
	 */
	clps_writel(0, INTMR1);
	clps_writel(0, INTMR2);

	/*
	 * Clear down any pending interrupts
	 */
	clps_writel(0, COEOI);
	clps_writel(0, TC1EOI);
	clps_writel(0, TC2EOI);
	clps_writel(0, RTCEOI);
	clps_writel(0, TEOI);
	clps_writel(0, UMSEOI);
	clps_writel(0, SYNCIO);
	clps_writel(0, KBDEOI);
}
