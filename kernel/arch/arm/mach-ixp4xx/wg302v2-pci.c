

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/irq.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>

#include <asm/mach/pci.h>

void __init wg302v2_pci_preinit(void)
{
	set_irq_type(IRQ_IXP4XX_GPIO8, IRQ_TYPE_LEVEL_LOW);
	set_irq_type(IRQ_IXP4XX_GPIO9, IRQ_TYPE_LEVEL_LOW);

	ixp4xx_pci_preinit();
}

static int __init wg302v2_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	if (slot == 1)
		return IRQ_IXP4XX_GPIO8;
	else if (slot == 2)
		return IRQ_IXP4XX_GPIO9;
	else return -1;
}

struct hw_pci wg302v2_pci __initdata = {
	.nr_controllers = 1,
	.preinit =        wg302v2_pci_preinit,
	.swizzle =        pci_std_swizzle,
	.setup =          ixp4xx_setup,
	.scan =           ixp4xx_scan_bus,
	.map_irq =        wg302v2_map_irq,
};

int __init wg302v2_pci_init(void)
{
	if (machine_is_wg302v2())
		pci_common_init(&wg302v2_pci);
	return 0;
}

subsys_initcall(wg302v2_pci_init);
