

#include <linux/init.h>

#include <msp_prom.h>
#include <msp_regs.h>

extern void msp_pci_init(void);

static int __init msp_pci_setup(void)
{
#if 0 /* Linux 2.6 initialization code to be completed */
	if (getdeviceid() & DEV_ID_SINGLE_PC) {
		/* If single card mode */
		slmRegs	*sreg = (slmRegs *) SREG_BASE;

		sreg->single_pc_enable = SINGLE_PCCARD;
	}
#endif

	msp_pci_init();

	return 0;
}

subsys_initcall(msp_pci_setup);
