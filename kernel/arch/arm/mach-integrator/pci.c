
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/init.h>

#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach/pci.h>
#include <asm/mach-types.h>


static u8 __init integrator_swizzle(struct pci_dev *dev, u8 *pinp)
{
	int pin = *pinp;

	if (pin == 0)
		pin = 1;

	while (dev->bus->self) {
		pin = pci_swizzle_interrupt_pin(dev, pin);
		/*
		 * move up the chain of bridges, swizzling as we go.
		 */
		dev = dev->bus->self;
	}
	*pinp = pin;

	return PCI_SLOT(dev->devfn);
}

static int irq_tab[4] __initdata = {
	IRQ_AP_PCIINT0,	IRQ_AP_PCIINT1,	IRQ_AP_PCIINT2,	IRQ_AP_PCIINT3
};

static int __init integrator_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	int intnr = ((slot - 9) + (pin - 1)) & 3;

	return irq_tab[intnr];
}

extern void pci_v3_init(void *);

static struct hw_pci integrator_pci __initdata = {
	.swizzle		= integrator_swizzle,
	.map_irq		= integrator_map_irq,
	.setup			= pci_v3_setup,
	.nr_controllers		= 1,
	.scan			= pci_v3_scan_bus,
	.preinit		= pci_v3_preinit,
	.postinit		= pci_v3_postinit,
};

static int __init integrator_pci_init(void)
{
	if (machine_is_integrator())
		pci_common_init(&integrator_pci);
	return 0;
}

subsys_initcall(integrator_pci_init);
