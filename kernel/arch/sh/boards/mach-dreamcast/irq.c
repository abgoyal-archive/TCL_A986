

#include <linux/irq.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <mach/sysasic.h>


#define ESR_BASE 0x005f6900    /* Base event status register */
#define EMR_BASE 0x005f6910    /* Base event mask register */

#define LEVEL(event) (((event) - HW_EVENT_IRQ_BASE) / 32)

/* Return the hardware event's bit positon within the EMR/ESR */
#define EVENT_BIT(event) (((event) - HW_EVENT_IRQ_BASE) & 31)


/* Disable the hardware event by masking its bit in its EMR */
static inline void disable_systemasic_irq(unsigned int irq)
{
	__u32 emr = EMR_BASE + (LEVEL(irq) << 4) + (LEVEL(irq) << 2);
	__u32 mask;

	mask = inl(emr);
	mask &= ~(1 << EVENT_BIT(irq));
	outl(mask, emr);
}

/* Enable the hardware event by setting its bit in its EMR */
static inline void enable_systemasic_irq(unsigned int irq)
{
	__u32 emr = EMR_BASE + (LEVEL(irq) << 4) + (LEVEL(irq) << 2);
	__u32 mask;

	mask = inl(emr);
	mask |= (1 << EVENT_BIT(irq));
	outl(mask, emr);
}

/* Acknowledge a hardware event by writing its bit back to its ESR */
static void mask_ack_systemasic_irq(unsigned int irq)
{
	__u32 esr = ESR_BASE + (LEVEL(irq) << 2);
	disable_systemasic_irq(irq);
	outl((1 << EVENT_BIT(irq)), esr);
}

struct irq_chip systemasic_int = {
	.name		= "System ASIC",
	.mask		= disable_systemasic_irq,
	.mask_ack	= mask_ack_systemasic_irq,
	.unmask		= enable_systemasic_irq,
};

int systemasic_irq_demux(int irq)
{
	__u32 emr, esr, status, level;
	__u32 j, bit;

	switch (irq) {
	case 13:
		level = 0;
		break;
	case 11:
		level = 1;
		break;
	case  9:
		level = 2;
		break;
	default:
		return irq;
	}
	emr = EMR_BASE + (level << 4) + (level << 2);
	esr = ESR_BASE + (level << 2);

	/* Mask the ESR to filter any spurious, unwanted interrupts */
	status = inl(esr);
	status &= inl(emr);

	/* Now scan and find the first set bit as the event to map */
	for (bit = 1, j = 0; j < 32; bit <<= 1, j++) {
		if (status & bit) {
			irq = HW_EVENT_IRQ_BASE + j + (level << 5);
			return irq;
		}
	}

	/* Not reached */
	return irq;
}

void systemasic_irq_init(void)
{
	int i, nid = cpu_to_node(boot_cpu_data);

	/* Assign all virtual IRQs to the System ASIC int. handler */
	for (i = HW_EVENT_IRQ_BASE; i < HW_EVENT_IRQ_MAX; i++) {
		unsigned int irq;

		irq = create_irq_nr(i, nid);
		if (unlikely(irq == 0)) {
			pr_err("%s: failed hooking irq %d for systemasic\n",
			       __func__, i);
			return;
		}

		if (unlikely(irq != i)) {
			pr_err("%s: got irq %d but wanted %d, bailing.\n",
			       __func__, irq, i);
			destroy_irq(irq);
			return;
		}

		set_irq_chip_and_handler(i, &systemasic_int,
					 handle_level_irq);
	}
}
