

#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/mipsregs.h>
#include <asm/txx9/generic.h>
#include <asm/txx9/rbtx4938.h>

static void toshiba_rbtx4938_irq_ioc_enable(unsigned int irq);
static void toshiba_rbtx4938_irq_ioc_disable(unsigned int irq);

#define TOSHIBA_RBTX4938_IOC_NAME "RBTX4938-IOC"
static struct irq_chip toshiba_rbtx4938_irq_ioc_type = {
	.name = TOSHIBA_RBTX4938_IOC_NAME,
	.ack = toshiba_rbtx4938_irq_ioc_disable,
	.mask = toshiba_rbtx4938_irq_ioc_disable,
	.mask_ack = toshiba_rbtx4938_irq_ioc_disable,
	.unmask = toshiba_rbtx4938_irq_ioc_enable,
};

static int toshiba_rbtx4938_irq_nested(int sw_irq)
{
	u8 level3;

	level3 = readb(rbtx4938_imstat_addr);
	if (unlikely(!level3))
		return -1;
	/* must use fls so onboard ATA has priority */
	return RBTX4938_IRQ_IOC + __fls8(level3);
}

static void __init
toshiba_rbtx4938_irq_ioc_init(void)
{
	int i;

	for (i = RBTX4938_IRQ_IOC;
	     i < RBTX4938_IRQ_IOC + RBTX4938_NR_IRQ_IOC; i++)
		set_irq_chip_and_handler(i, &toshiba_rbtx4938_irq_ioc_type,
					 handle_level_irq);

	set_irq_chained_handler(RBTX4938_IRQ_IOCINT, handle_simple_irq);
}

static void
toshiba_rbtx4938_irq_ioc_enable(unsigned int irq)
{
	unsigned char v;

	v = readb(rbtx4938_imask_addr);
	v |= (1 << (irq - RBTX4938_IRQ_IOC));
	writeb(v, rbtx4938_imask_addr);
	mmiowb();
}

static void
toshiba_rbtx4938_irq_ioc_disable(unsigned int irq)
{
	unsigned char v;

	v = readb(rbtx4938_imask_addr);
	v &= ~(1 << (irq - RBTX4938_IRQ_IOC));
	writeb(v, rbtx4938_imask_addr);
	mmiowb();
}

static int rbtx4938_irq_dispatch(int pending)
{
	int irq;

	if (pending & STATUSF_IP7)
		irq = MIPS_CPU_IRQ_BASE + 7;
	else if (pending & STATUSF_IP2) {
		irq = txx9_irq();
		if (irq == RBTX4938_IRQ_IOCINT)
			irq = toshiba_rbtx4938_irq_nested(irq);
	} else if (pending & STATUSF_IP1)
		irq = MIPS_CPU_IRQ_BASE + 0;
	else if (pending & STATUSF_IP0)
		irq = MIPS_CPU_IRQ_BASE + 1;
	else
		irq = -1;
	return irq;
}

void __init rbtx4938_irq_setup(void)
{
	txx9_irq_dispatch = rbtx4938_irq_dispatch;
	/* Now, interrupt control disabled, */
	/* all IRC interrupts are masked, */
	/* all IRC interrupt mode are Low Active. */

	/* mask all IOC interrupts */
	writeb(0, rbtx4938_imask_addr);

	/* clear SoftInt interrupts */
	writeb(0, rbtx4938_softint_addr);
	tx4938_irq_init();
	toshiba_rbtx4938_irq_ioc_init();
	/* Onboard 10M Ether: High Active */
	set_irq_type(RBTX4938_IRQ_ETHER, IRQF_TRIGGER_HIGH);
}
