
#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <asm/processor.h>
#include <asm/intctl-regs.h>
#include <asm/serial-regs.h>
#include <unit/serial.h>

asmlinkage void __init unit_init(void)
{
#ifndef CONFIG_GDBSTUB_ON_TTYSx
	/* set the 16550 interrupt line to level 3 if not being used for GDB */
	set_intr_level(XIRQ0, GxICR_LEVEL_3);
#endif
}

void __init unit_setup(void)
{
#ifdef CONFIG_PCI
	unit_pci_init();
#endif
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
