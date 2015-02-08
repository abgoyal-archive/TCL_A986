

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/prom.h>
#include <asm/irq.h>

#include "gef_pic.h"

#define DEBUG
#undef DEBUG

#ifdef DEBUG
#define DBG(fmt...) do { printk(KERN_DEBUG "gef_pic: " fmt); } while (0)
#else
#define DBG(fmt...) do { } while (0)
#endif

#define GEF_PIC_NUM_IRQS	32

/* Interrupt Controller Interface Registers */
#define GEF_PIC_INTR_STATUS	0x0000

#define GEF_PIC_INTR_MASK(cpu)	(0x0010 + (0x4 * cpu))
#define GEF_PIC_CPU0_INTR_MASK	GEF_PIC_INTR_MASK(0)
#define GEF_PIC_CPU1_INTR_MASK	GEF_PIC_INTR_MASK(1)

#define GEF_PIC_MCP_MASK(cpu)	(0x0018 + (0x4 * cpu))
#define GEF_PIC_CPU0_MCP_MASK	GEF_PIC_MCP_MASK(0)
#define GEF_PIC_CPU1_MCP_MASK	GEF_PIC_MCP_MASK(1)

#define gef_irq_to_hw(virq)    ((unsigned int)irq_map[virq].hwirq)


static DEFINE_RAW_SPINLOCK(gef_pic_lock);

static void __iomem *gef_pic_irq_reg_base;
static struct irq_host *gef_pic_irq_host;
static int gef_pic_cascade_irq;


void gef_pic_cascade(unsigned int irq, struct irq_desc *desc)
{
	unsigned int cascade_irq;

	/*
	 * See if we actually have an interrupt, call generic handling code if
	 * we do.
	 */
	cascade_irq = gef_pic_get_irq();

	if (cascade_irq != NO_IRQ)
		generic_handle_irq(cascade_irq);

	desc->chip->eoi(irq);

}

static void gef_pic_mask(unsigned int virq)
{
	unsigned long flags;
	unsigned int hwirq;
	u32 mask;

	hwirq = gef_irq_to_hw(virq);

	raw_spin_lock_irqsave(&gef_pic_lock, flags);
	mask = in_be32(gef_pic_irq_reg_base + GEF_PIC_INTR_MASK(0));
	mask &= ~(1 << hwirq);
	out_be32(gef_pic_irq_reg_base + GEF_PIC_INTR_MASK(0), mask);
	raw_spin_unlock_irqrestore(&gef_pic_lock, flags);
}

static void gef_pic_mask_ack(unsigned int virq)
{
	/* Don't think we actually have to do anything to ack an interrupt,
	 * we just need to clear down the devices interrupt and it will go away
	 */
	gef_pic_mask(virq);
}

static void gef_pic_unmask(unsigned int virq)
{
	unsigned long flags;
	unsigned int hwirq;
	u32 mask;

	hwirq = gef_irq_to_hw(virq);

	raw_spin_lock_irqsave(&gef_pic_lock, flags);
	mask = in_be32(gef_pic_irq_reg_base + GEF_PIC_INTR_MASK(0));
	mask |= (1 << hwirq);
	out_be32(gef_pic_irq_reg_base + GEF_PIC_INTR_MASK(0), mask);
	raw_spin_unlock_irqrestore(&gef_pic_lock, flags);
}

static struct irq_chip gef_pic_chip = {
	.name		= "gefp",
	.mask		= gef_pic_mask,
	.mask_ack	= gef_pic_mask_ack,
	.unmask		= gef_pic_unmask,
};


static int gef_pic_host_map(struct irq_host *h, unsigned int virq,
			  irq_hw_number_t hwirq)
{
	/* All interrupts are LEVEL sensitive */
	irq_to_desc(virq)->status |= IRQ_LEVEL;
	set_irq_chip_and_handler(virq, &gef_pic_chip, handle_level_irq);

	return 0;
}

static int gef_pic_host_xlate(struct irq_host *h, struct device_node *ct,
			    const u32 *intspec, unsigned int intsize,
			    irq_hw_number_t *out_hwirq, unsigned int *out_flags)
{

	*out_hwirq = intspec[0];
	if (intsize > 1)
		*out_flags = intspec[1];
	else
		*out_flags = IRQ_TYPE_LEVEL_HIGH;

	return 0;
}

static struct irq_host_ops gef_pic_host_ops = {
	.map	= gef_pic_host_map,
	.xlate	= gef_pic_host_xlate,
};


void __init gef_pic_init(struct device_node *np)
{
	unsigned long flags;

	/* Map the devices registers into memory */
	gef_pic_irq_reg_base = of_iomap(np, 0);

	raw_spin_lock_irqsave(&gef_pic_lock, flags);

	/* Initialise everything as masked. */
	out_be32(gef_pic_irq_reg_base + GEF_PIC_CPU0_INTR_MASK, 0);
	out_be32(gef_pic_irq_reg_base + GEF_PIC_CPU1_INTR_MASK, 0);

	out_be32(gef_pic_irq_reg_base + GEF_PIC_CPU0_MCP_MASK, 0);
	out_be32(gef_pic_irq_reg_base + GEF_PIC_CPU1_MCP_MASK, 0);

	raw_spin_unlock_irqrestore(&gef_pic_lock, flags);

	/* Map controller */
	gef_pic_cascade_irq = irq_of_parse_and_map(np, 0);
	if (gef_pic_cascade_irq == NO_IRQ) {
		printk(KERN_ERR "SBC610: failed to map cascade interrupt");
		return;
	}

	/* Setup an irq_host structure */
	gef_pic_irq_host = irq_alloc_host(np, IRQ_HOST_MAP_LINEAR,
					  GEF_PIC_NUM_IRQS,
					  &gef_pic_host_ops, NO_IRQ);
	if (gef_pic_irq_host == NULL)
		return;

	/* Chain with parent controller */
	set_irq_chained_handler(gef_pic_cascade_irq, gef_pic_cascade);
}

unsigned int gef_pic_get_irq(void)
{
	u32 cause, mask, active;
	unsigned int virq = NO_IRQ;
	int hwirq;

	cause = in_be32(gef_pic_irq_reg_base + GEF_PIC_INTR_STATUS);

	mask = in_be32(gef_pic_irq_reg_base + GEF_PIC_INTR_MASK(0));

	active = cause & mask;

	if (active) {
		for (hwirq = GEF_PIC_NUM_IRQS - 1; hwirq > -1; hwirq--) {
			if (active & (0x1 << hwirq))
				break;
		}
		virq = irq_linear_revmap(gef_pic_irq_host,
			(irq_hw_number_t)hwirq);
	}

	return virq;
}

