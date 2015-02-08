

#include <linux/init.h>
#include <linux/interrupt.h>

#include <asm/mach-au1x00/au1000.h>
#include <asm/mach-pb1x00/pb1550.h>
#include <asm/mach-db1x00/bcsr.h>
#include <asm/mach-au1x00/gpio.h>

#include <prom.h>


char irq_tab_alchemy[][5] __initdata = {
	[12] = { -1, AU1550_PCI_INTB, AU1550_PCI_INTC, AU1550_PCI_INTD, AU1550_PCI_INTA }, /* IDSEL 12 - PCI slot 2 (left)  */
	[13] = { -1, AU1550_PCI_INTA, AU1550_PCI_INTB, AU1550_PCI_INTC, AU1550_PCI_INTD }, /* IDSEL 13 - PCI slot 1 (right) */
};

const char *get_system_type(void)
{
	return "Alchemy Pb1550";
}

void __init board_setup(void)
{
	u32 pin_func;

	bcsr_init(PB1550_BCSR_PHYS_ADDR,
		  PB1550_BCSR_PHYS_ADDR + PB1550_BCSR_HEXLED_OFS);

	alchemy_gpio2_enable();

	/*
	 * Enable PSC1 SYNC for AC'97.  Normaly done in audio driver,
	 * but it is board specific code, so put it here.
	 */
	pin_func = au_readl(SYS_PINFUNC);
	au_sync();
	pin_func |= SYS_PF_MUST_BE_SET | SYS_PF_PSC1_S1;
	au_writel(pin_func, SYS_PINFUNC);

	bcsr_write(BCSR_PCMCIA, 0);	/* turn off PCMCIA power */

	printk(KERN_INFO "AMD Alchemy Pb1550 Board\n");
}

static int __init pb1550_init_irq(void)
{
	set_irq_type(AU1550_GPIO0_INT, IRQF_TRIGGER_LOW);
	set_irq_type(AU1550_GPIO1_INT, IRQF_TRIGGER_LOW);
	set_irq_type(AU1550_GPIO201_205_INT, IRQF_TRIGGER_HIGH);

	/* enable both PCMCIA card irqs in the shared line */
	alchemy_gpio2_enable_int(201);
	alchemy_gpio2_enable_int(202);

	return 0;
}
arch_initcall(pb1550_init_irq);
