

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/irq.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>

#include <asm/mach/pci.h>

void __init ixdpg425_pci_preinit(void)
{
	set_irq_type(IRQ_IXP4XX_GPIO6, IRQ_TYPE_LEVEL_LOW);
	set_irq_type(IRQ_IXP4XX_GPIO7, IRQ_TYPE_LEVEL_LOW);

	ixp4xx_pci_preinit();
}

static int __init ixdpg425_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	if (slot == 12 || slot == 13)
		return IRQ_IXP4XX_GPIO7;
	else if (slot == 14)
		return IRQ_IXP4XX_GPIO6;
	else return -1;
}

struct hw_pci ixdpg425_pci __initdata = {
	.nr_controllers = 1,
	.preinit =        ixdpg425_pci_preinit,
	.swizzle =        pci_std_swizzle,
	.setup =          ixp4xx_setup,
	.scan =           ixp4xx_scan_bus,
	.map_irq =        ixdpg425_map_irq,
};

int __init ixdpg425_pci_init(void)
{
	if (machine_is_ixdpg425())
		pci_common_init(&ixdpg425_pci);
	return 0;
}

subsys_initcall(ixdpg425_pci_init);
