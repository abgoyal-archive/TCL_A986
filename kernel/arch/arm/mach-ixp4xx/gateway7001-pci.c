

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/irq.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>

#include <asm/mach/pci.h>

void __init gateway7001_pci_preinit(void)
{
	set_irq_type(IRQ_IXP4XX_GPIO10, IRQ_TYPE_LEVEL_LOW);
	set_irq_type(IRQ_IXP4XX_GPIO11, IRQ_TYPE_LEVEL_LOW);

	ixp4xx_pci_preinit();
}

static int __init gateway7001_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	if (slot == 1)
		return IRQ_IXP4XX_GPIO11;
	else if (slot == 2)
		return IRQ_IXP4XX_GPIO10;
	else return -1;
}

struct hw_pci gateway7001_pci __initdata = {
	.nr_controllers = 1,
	.preinit =        gateway7001_pci_preinit,
	.swizzle =        pci_std_swizzle,
	.setup =          ixp4xx_setup,
	.scan =           ixp4xx_scan_bus,
	.map_irq =        gateway7001_map_irq,
};

int __init gateway7001_pci_init(void)
{
	if (machine_is_gateway7001())
		pci_common_init(&gateway7001_pci);
	return 0;
}

subsys_initcall(gateway7001_pci_init);
