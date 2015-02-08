

#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/pm.h>

#include <asm/reboot.h>
#include <asm/mach-au1x00/au1000.h>

#include <prom.h>

char irq_tab_alchemy[][5] __initdata = {
	[0] = { -1, AU1500_PCI_INTA, AU1500_PCI_INTA, 0xff, 0xff }, /* IDSEL 00 - AdapterA-Slot0 (top) */
	[1] = { -1, AU1500_PCI_INTB, AU1500_PCI_INTA, 0xff, 0xff }, /* IDSEL 01 - AdapterA-Slot1 (bottom) */
	[2] = { -1, AU1500_PCI_INTC, AU1500_PCI_INTD, 0xff, 0xff }, /* IDSEL 02 - AdapterB-Slot0 (top) */
	[3] = { -1, AU1500_PCI_INTD, AU1500_PCI_INTC, 0xff, 0xff }, /* IDSEL 03 - AdapterB-Slot1 (bottom) */
	[4] = { -1, AU1500_PCI_INTA, AU1500_PCI_INTB, 0xff, 0xff }, /* IDSEL 04 - AdapterC-Slot0 (top) */
	[5] = { -1, AU1500_PCI_INTB, AU1500_PCI_INTA, 0xff, 0xff }, /* IDSEL 05 - AdapterC-Slot1 (bottom) */
	[6] = { -1, AU1500_PCI_INTC, AU1500_PCI_INTD, 0xff, 0xff }, /* IDSEL 06 - AdapterD-Slot0 (top) */
	[7] = { -1, AU1500_PCI_INTD, AU1500_PCI_INTC, 0xff, 0xff }, /* IDSEL 07 - AdapterD-Slot1 (bottom) */
};

extern int (*board_pci_idsel)(unsigned int devsel, int assert);
int mtx1_pci_idsel(unsigned int devsel, int assert);

static void mtx1_reset(char *c)
{
	/* Hit BCSR.SYSTEM_CONTROL[SW_RST] */
	au_writel(0x00000000, 0xAE00001C);
}

static void mtx1_power_off(void)
{
	printk(KERN_ALERT "It's now safe to remove power\n");
	while (1)
		asm volatile (".set mips3 ; wait ; .set mips1");
}

void __init board_setup(void)
{
#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
	/* Enable USB power switch */
	alchemy_gpio_direction_output(204, 0);
#endif /* defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE) */

#ifdef CONFIG_PCI
#if defined(__MIPSEB__)
	au_writel(0xf | (2 << 6) | (1 << 4), Au1500_PCI_CFG);
#else
	au_writel(0xf, Au1500_PCI_CFG);
#endif
	board_pci_idsel = mtx1_pci_idsel;
#endif

	/* Initialize sys_pinfunc */
	au_writel(SYS_PF_NI2, SYS_PINFUNC);

	/* Initialize GPIO */
	au_writel(0xFFFFFFFF, SYS_TRIOUTCLR);
	alchemy_gpio_direction_output(0, 0);	/* Disable M66EN (PCI 66MHz) */
	alchemy_gpio_direction_output(3, 1);	/* Disable PCI CLKRUN# */
	alchemy_gpio_direction_output(1, 1);	/* Enable EXT_IO3 */
	alchemy_gpio_direction_output(5, 0);	/* Disable eth PHY TX_ER */

	/* Enable LED and set it to green */
	alchemy_gpio_direction_output(211, 1);	/* green on */
	alchemy_gpio_direction_output(212, 0);	/* red off */

	pm_power_off = mtx1_power_off;
	_machine_halt = mtx1_power_off;
	_machine_restart = mtx1_reset;

	printk(KERN_INFO "4G Systems MTX-1 Board\n");
}

int
mtx1_pci_idsel(unsigned int devsel, int assert)
{
#define MTX_IDSEL_ONLY_0_AND_3 0
#if MTX_IDSEL_ONLY_0_AND_3
	if (devsel != 0 && devsel != 3) {
		printk(KERN_ERR "*** not 0 or 3\n");
		return 0;
	}
#endif

	if (assert && devsel != 0)
		/* Suppress signal to Cardbus */
		alchemy_gpio_set_value(1, 0);	/* set EXT_IO3 OFF */
	else
		alchemy_gpio_set_value(1, 1);	/* set EXT_IO3 ON */

	udelay(1);
	return 1;
}

static int __init mtx1_init_irq(void)
{
	set_irq_type(AU1500_GPIO204_INT, IRQF_TRIGGER_HIGH);
	set_irq_type(AU1500_GPIO201_INT, IRQF_TRIGGER_LOW);
	set_irq_type(AU1500_GPIO202_INT, IRQF_TRIGGER_LOW);
	set_irq_type(AU1500_GPIO203_INT, IRQF_TRIGGER_LOW);
	set_irq_type(AU1500_GPIO205_INT, IRQF_TRIGGER_LOW);

	return 0;
}
arch_initcall(mtx1_init_irq);
