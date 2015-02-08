

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/types.h>

#include <asm/dec/kn02.h>


u32 cached_kn02_csr;


static int kn02_irq_base;


static inline void unmask_kn02_irq(unsigned int irq)
{
	volatile u32 *csr = (volatile u32 *)CKSEG1ADDR(KN02_SLOT_BASE +
						       KN02_CSR);

	cached_kn02_csr |= (1 << (irq - kn02_irq_base + 16));
	*csr = cached_kn02_csr;
}

static inline void mask_kn02_irq(unsigned int irq)
{
	volatile u32 *csr = (volatile u32 *)CKSEG1ADDR(KN02_SLOT_BASE +
						       KN02_CSR);

	cached_kn02_csr &= ~(1 << (irq - kn02_irq_base + 16));
	*csr = cached_kn02_csr;
}

static void ack_kn02_irq(unsigned int irq)
{
	mask_kn02_irq(irq);
	iob();
}

static struct irq_chip kn02_irq_type = {
	.name = "KN02-CSR",
	.ack = ack_kn02_irq,
	.mask = mask_kn02_irq,
	.mask_ack = ack_kn02_irq,
	.unmask = unmask_kn02_irq,
};


void __init init_kn02_irqs(int base)
{
	volatile u32 *csr = (volatile u32 *)CKSEG1ADDR(KN02_SLOT_BASE +
						       KN02_CSR);
	int i;

	/* Mask interrupts. */
	cached_kn02_csr &= ~KN02_CSR_IOINTEN;
	*csr = cached_kn02_csr;
	iob();

	for (i = base; i < base + KN02_IRQ_LINES; i++)
		set_irq_chip_and_handler(i, &kn02_irq_type, handle_level_irq);

	kn02_irq_base = base;
}
