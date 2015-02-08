

#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <asm/traps.h>
#include <asm/coldfire.h>
#include <asm/mcfsim.h>

unsigned char mcf_irq2imr[NR_IRQS];

#define	EIRQ1	25
#define	EIRQ7	31

#ifdef MCFSIM_IMR_IS_16BITS

void mcf_setimr(int index)
{
	u16 imr;
	imr = __raw_readw(MCF_MBAR + MCFSIM_IMR);
	__raw_writew(imr | (0x1 << index), MCF_MBAR + MCFSIM_IMR);
}

void mcf_clrimr(int index)
{
	u16 imr;
	imr = __raw_readw(MCF_MBAR + MCFSIM_IMR);
	__raw_writew(imr & ~(0x1 << index), MCF_MBAR + MCFSIM_IMR);
}

void mcf_maskimr(unsigned int mask)
{
	u16 imr;
	imr = __raw_readw(MCF_MBAR + MCFSIM_IMR);
	imr |= mask;
	__raw_writew(imr, MCF_MBAR + MCFSIM_IMR);
}

#else

void mcf_setimr(int index)
{
	u32 imr;
	imr = __raw_readl(MCF_MBAR + MCFSIM_IMR);
	__raw_writel(imr | (0x1 << index), MCF_MBAR + MCFSIM_IMR);
}

void mcf_clrimr(int index)
{
	u32 imr;
	imr = __raw_readl(MCF_MBAR + MCFSIM_IMR);
	__raw_writel(imr & ~(0x1 << index), MCF_MBAR + MCFSIM_IMR);
}

void mcf_maskimr(unsigned int mask)
{
	u32 imr;
	imr = __raw_readl(MCF_MBAR + MCFSIM_IMR);
	imr |= mask;
	__raw_writel(imr, MCF_MBAR + MCFSIM_IMR);
}

#endif

void mcf_autovector(int irq)
{
#ifdef MCFSIM_AVR
	if ((irq >= EIRQ1) && (irq <= EIRQ7)) {
		u8 avec;
		avec = __raw_readb(MCF_MBAR + MCFSIM_AVR);
		avec |= (0x1 << (irq - EIRQ1 + 1));
		__raw_writeb(avec, MCF_MBAR + MCFSIM_AVR);
	}
#endif
}

static void intc_irq_mask(unsigned int irq)
{
	if (mcf_irq2imr[irq])
		mcf_setimr(mcf_irq2imr[irq]);
}

static void intc_irq_unmask(unsigned int irq)
{
	if (mcf_irq2imr[irq])
		mcf_clrimr(mcf_irq2imr[irq]);
}

static int intc_irq_set_type(unsigned int irq, unsigned int type)
{
	return 0;
}

static struct irq_chip intc_irq_chip = {
	.name		= "CF-INTC",
	.mask		= intc_irq_mask,
	.unmask		= intc_irq_unmask,
	.set_type	= intc_irq_set_type,
};

void __init init_IRQ(void)
{
	int irq;

	init_vectors();
	mcf_maskimr(0xffffffff);

	for (irq = 0; (irq < NR_IRQS); irq++) {
		irq_desc[irq].status = IRQ_DISABLED;
		irq_desc[irq].action = NULL;
		irq_desc[irq].depth = 1;
		irq_desc[irq].chip = &intc_irq_chip;
		intc_irq_set_type(irq, 0);
	}
}

