

#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/device.h>

#include <asm/io.h>
#include <asm/setup.h>
#include <asm/processor.h>
#include <asm/irq.h>
#include <asm/intctl-regs.h>

asmlinkage void __init unit_init(void)
{
	/* set up the external interrupts */
	SET_XIRQ_TRIGGER(0, XIRQ_TRIGGER_HILEVEL);
	SET_XIRQ_TRIGGER(2, XIRQ_TRIGGER_LOWLEVEL);
	SET_XIRQ_TRIGGER(3, XIRQ_TRIGGER_HILEVEL);
	SET_XIRQ_TRIGGER(4, XIRQ_TRIGGER_LOWLEVEL);
	SET_XIRQ_TRIGGER(5, XIRQ_TRIGGER_LOWLEVEL);
}

void __init unit_setup(void)
{
}

void __init unit_init_IRQ(void)
{
	unsigned int extnum;

	for (extnum = 0; extnum < NR_XIRQS; extnum++) {
		switch (GET_XIRQ_TRIGGER(extnum)) {
		case XIRQ_TRIGGER_HILEVEL:
		case XIRQ_TRIGGER_LOWLEVEL:
			set_intr_postackable(XIRQ2IRQ(extnum));
			break;
		default:
			break;
		}
	}
}
