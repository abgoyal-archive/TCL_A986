
#include <linux/init.h>
#include <linux/types.h>
#include <linux/interrupt.h>

#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/txx9/generic.h>
#include <asm/txx9/jmr3927.h>

#if JMR3927_IRQ_END > NR_IRQS
#error JMR3927_IRQ_END > NR_IRQS
#endif

static void mask_irq_ioc(unsigned int irq)
{
	/* 0: mask */
	unsigned int irq_nr = irq - JMR3927_IRQ_IOC;
	unsigned char imask = jmr3927_ioc_reg_in(JMR3927_IOC_INTM_ADDR);
	unsigned int bit = 1 << irq_nr;
	jmr3927_ioc_reg_out(imask & ~bit, JMR3927_IOC_INTM_ADDR);
	/* flush write buffer */
	(void)jmr3927_ioc_reg_in(JMR3927_IOC_REV_ADDR);
}
static void unmask_irq_ioc(unsigned int irq)
{
	/* 0: mask */
	unsigned int irq_nr = irq - JMR3927_IRQ_IOC;
	unsigned char imask = jmr3927_ioc_reg_in(JMR3927_IOC_INTM_ADDR);
	unsigned int bit = 1 << irq_nr;
	jmr3927_ioc_reg_out(imask | bit, JMR3927_IOC_INTM_ADDR);
	/* flush write buffer */
	(void)jmr3927_ioc_reg_in(JMR3927_IOC_REV_ADDR);
}

static int jmr3927_ioc_irqroute(void)
{
	unsigned char istat = jmr3927_ioc_reg_in(JMR3927_IOC_INTS2_ADDR);
	int i;

	for (i = 0; i < JMR3927_NR_IRQ_IOC; i++) {
		if (istat & (1 << i))
			return JMR3927_IRQ_IOC + i;
	}
	return -1;
}

static int jmr3927_irq_dispatch(int pending)
{
	int irq;

	if ((pending & CAUSEF_IP7) == 0)
		return -1;
	irq = (pending >> CAUSEB_IP2) & 0x0f;
	irq += JMR3927_IRQ_IRC;
	if (irq == JMR3927_IRQ_IOCINT)
		irq = jmr3927_ioc_irqroute();
	return irq;
}

static struct irq_chip jmr3927_irq_ioc = {
	.name = "jmr3927_ioc",
	.ack = mask_irq_ioc,
	.mask = mask_irq_ioc,
	.mask_ack = mask_irq_ioc,
	.unmask = unmask_irq_ioc,
};

void __init jmr3927_irq_setup(void)
{
	int i;

	txx9_irq_dispatch = jmr3927_irq_dispatch;
	/* Now, interrupt control disabled, */
	/* all IRC interrupts are masked, */
	/* all IRC interrupt mode are Low Active. */

	/* mask all IOC interrupts */
	jmr3927_ioc_reg_out(0, JMR3927_IOC_INTM_ADDR);
	/* setup IOC interrupt mode (SOFT:High Active, Others:Low Active) */
	jmr3927_ioc_reg_out(JMR3927_IOC_INTF_SOFT, JMR3927_IOC_INTP_ADDR);

	/* clear PCI Soft interrupts */
	jmr3927_ioc_reg_out(0, JMR3927_IOC_INTS1_ADDR);
	/* clear PCI Reset interrupts */
	jmr3927_ioc_reg_out(0, JMR3927_IOC_RESET_ADDR);

	tx3927_irq_init();
	for (i = JMR3927_IRQ_IOC; i < JMR3927_IRQ_IOC + JMR3927_NR_IRQ_IOC; i++)
		set_irq_chip_and_handler(i, &jmr3927_irq_ioc, handle_level_irq);

	/* setup IOC interrupt 1 (PCI, MODEM) */
	set_irq_chained_handler(JMR3927_IRQ_IOCINT, handle_simple_irq);
}
