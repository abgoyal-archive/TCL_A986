

#include <linux/init.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/txx9/generic.h>
#include <asm/txx9/rbtx4927.h>

static void toshiba_rbtx4927_irq_ioc_enable(unsigned int irq);
static void toshiba_rbtx4927_irq_ioc_disable(unsigned int irq);

#define TOSHIBA_RBTX4927_IOC_NAME "RBTX4927-IOC"
static struct irq_chip toshiba_rbtx4927_irq_ioc_type = {
	.name = TOSHIBA_RBTX4927_IOC_NAME,
	.ack = toshiba_rbtx4927_irq_ioc_disable,
	.mask = toshiba_rbtx4927_irq_ioc_disable,
	.mask_ack = toshiba_rbtx4927_irq_ioc_disable,
	.unmask = toshiba_rbtx4927_irq_ioc_enable,
};

static int toshiba_rbtx4927_irq_nested(int sw_irq)
{
	u8 level3;

	level3 = readb(rbtx4927_imstat_addr) & 0x1f;
	if (unlikely(!level3))
		return -1;
	return RBTX4927_IRQ_IOC + __fls8(level3);
}

static void __init toshiba_rbtx4927_irq_ioc_init(void)
{
	int i;

	/* mask all IOC interrupts */
	writeb(0, rbtx4927_imask_addr);
	/* clear SoftInt interrupts */
	writeb(0, rbtx4927_softint_addr);

	for (i = RBTX4927_IRQ_IOC;
	     i < RBTX4927_IRQ_IOC + RBTX4927_NR_IRQ_IOC; i++)
		set_irq_chip_and_handler(i, &toshiba_rbtx4927_irq_ioc_type,
					 handle_level_irq);
	set_irq_chained_handler(RBTX4927_IRQ_IOCINT, handle_simple_irq);
}

static void toshiba_rbtx4927_irq_ioc_enable(unsigned int irq)
{
	unsigned char v;

	v = readb(rbtx4927_imask_addr);
	v |= (1 << (irq - RBTX4927_IRQ_IOC));
	writeb(v, rbtx4927_imask_addr);
}

static void toshiba_rbtx4927_irq_ioc_disable(unsigned int irq)
{
	unsigned char v;

	v = readb(rbtx4927_imask_addr);
	v &= ~(1 << (irq - RBTX4927_IRQ_IOC));
	writeb(v, rbtx4927_imask_addr);
	mmiowb();
}


static int rbtx4927_irq_dispatch(int pending)
{
	int irq;

	if (pending & STATUSF_IP7)			/* cpu timer */
		irq = MIPS_CPU_IRQ_BASE + 7;
	else if (pending & STATUSF_IP2) {		/* tx4927 pic */
		irq = txx9_irq();
		if (irq == RBTX4927_IRQ_IOCINT)
			irq = toshiba_rbtx4927_irq_nested(irq);
	} else if (pending & STATUSF_IP0)		/* user line 0 */
		irq = MIPS_CPU_IRQ_BASE + 0;
	else if (pending & STATUSF_IP1)			/* user line 1 */
		irq = MIPS_CPU_IRQ_BASE + 1;
	else
		irq = -1;
	return irq;
}

void __init rbtx4927_irq_setup(void)
{
	txx9_irq_dispatch = rbtx4927_irq_dispatch;
	tx4927_irq_init();
	toshiba_rbtx4927_irq_ioc_init();
	/* Onboard 10M Ether: High Active */
	set_irq_type(RBTX4927_RTL_8019_IRQ, IRQF_TRIGGER_HIGH);
}
