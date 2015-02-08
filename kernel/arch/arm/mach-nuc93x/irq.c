

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/ptrace.h>
#include <linux/sysdev.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/regs-irq.h>

static void nuc93x_irq_mask(unsigned int irq)
{
	__raw_writel(1 << irq, REG_AIC_MDCR);
}


static void nuc93x_irq_ack(unsigned int irq)
{
	__raw_writel(0x01, REG_AIC_EOSCR);
}

static void nuc93x_irq_unmask(unsigned int irq)
{
	__raw_writel(1 << irq, REG_AIC_MECR);

}

static struct irq_chip nuc93x_irq_chip = {
	.ack	   = nuc93x_irq_ack,
	.mask	   = nuc93x_irq_mask,
	.unmask	   = nuc93x_irq_unmask,
};

void __init nuc93x_init_irq(void)
{
	int irqno;

	__raw_writel(0xFFFFFFFE, REG_AIC_MDCR);

	for (irqno = IRQ_WDT; irqno <= NR_IRQS; irqno++) {
		set_irq_chip(irqno, &nuc93x_irq_chip);
		set_irq_handler(irqno, handle_level_irq);
		set_irq_flags(irqno, IRQF_VALID);
	}
}
