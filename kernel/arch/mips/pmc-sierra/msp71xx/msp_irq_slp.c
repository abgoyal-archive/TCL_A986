

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/bitops.h>

#include <asm/mipsregs.h>
#include <asm/system.h>

#include <msp_slp_int.h>
#include <msp_regs.h>

static inline void unmask_msp_slp_irq(unsigned int irq)
{
	/* check for PER interrupt range */
	if (irq < MSP_PER_INTBASE)
		*SLP_INT_MSK_REG |= (1 << (irq - MSP_SLP_INTBASE));
	else
		*PER_INT_MSK_REG |= (1 << (irq - MSP_PER_INTBASE));
}

static inline void mask_msp_slp_irq(unsigned int irq)
{
	/* check for PER interrupt range */
	if (irq < MSP_PER_INTBASE)
		*SLP_INT_MSK_REG &= ~(1 << (irq - MSP_SLP_INTBASE));
	else
		*PER_INT_MSK_REG &= ~(1 << (irq - MSP_PER_INTBASE));
}

static inline void ack_msp_slp_irq(unsigned int irq)
{
	/* check for PER interrupt range */
	if (irq < MSP_PER_INTBASE)
		*SLP_INT_STS_REG = (1 << (irq - MSP_SLP_INTBASE));
	else
		*PER_INT_STS_REG = (1 << (irq - MSP_PER_INTBASE));
}

static struct irq_chip msp_slp_irq_controller = {
	.name = "MSP_SLP",
	.ack = ack_msp_slp_irq,
	.mask = mask_msp_slp_irq,
	.unmask = unmask_msp_slp_irq,
};

void __init msp_slp_irq_init(void)
{
	int i;

	/* Mask/clear interrupts. */
	*SLP_INT_MSK_REG = 0x00000000;
	*PER_INT_MSK_REG = 0x00000000;
	*SLP_INT_STS_REG = 0xFFFFFFFF;
	*PER_INT_STS_REG = 0xFFFFFFFF;

	/* initialize all the IRQ descriptors */
	for (i = MSP_SLP_INTBASE; i < MSP_PER_INTBASE + 32; i++)
		set_irq_chip_and_handler(i, &msp_slp_irq_controller,
					 handle_level_irq);
}

void msp_slp_irq_dispatch(void)
{
	u32 pending;
	int intbase;

	intbase = MSP_SLP_INTBASE;
	pending = *SLP_INT_STS_REG & *SLP_INT_MSK_REG;

	/* check for PER interrupt */
	if (pending == (1 << (MSP_INT_PER - MSP_SLP_INTBASE))) {
		intbase = MSP_PER_INTBASE;
		pending = *PER_INT_STS_REG & *PER_INT_MSK_REG;
	}

	/* check for spurious interrupt */
	if (pending == 0x00000000) {
		printk(KERN_ERR "Spurious %s interrupt?\n",
			(intbase == MSP_SLP_INTBASE) ? "SLP" : "PER");
		return;
	}

	/* dispatch the irq */
	do_IRQ(ffs(pending) + intbase - 1);
}
