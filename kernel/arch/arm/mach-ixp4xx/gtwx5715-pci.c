

#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <asm/mach/pci.h>

#define SLOT0_DEVID	0
#define SLOT1_DEVID	1
#define INTA		10 /* slot 1 has INTA and INTB crossed */
#define INTB		11

void __init gtwx5715_pci_preinit(void)
{
	set_irq_type(IXP4XX_GPIO_IRQ(INTA), IRQ_TYPE_LEVEL_LOW);
	set_irq_type(IXP4XX_GPIO_IRQ(INTB), IRQ_TYPE_LEVEL_LOW);
	ixp4xx_pci_preinit();
}


static int __init gtwx5715_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	int rc = -1;

	if ((slot == SLOT0_DEVID && pin == 1) ||
	    (slot == SLOT1_DEVID && pin == 2))
		rc = IXP4XX_GPIO_IRQ(INTA);
	else if ((slot == SLOT0_DEVID && pin == 2) ||
		 (slot == SLOT1_DEVID && pin == 1))
		rc = IXP4XX_GPIO_IRQ(INTB);

	printk(KERN_INFO "%s: Mapped slot %d pin %d to IRQ %d\n",
	       __func__, slot, pin, rc);
	return rc;
}

struct hw_pci gtwx5715_pci __initdata = {
	.nr_controllers = 1,
	.preinit =        gtwx5715_pci_preinit,
	.swizzle =        pci_std_swizzle,
	.setup =          ixp4xx_setup,
	.scan =           ixp4xx_scan_bus,
	.map_irq =        gtwx5715_map_irq,
};

int __init gtwx5715_pci_init(void)
{
	if (machine_is_gtwx5715())
		pci_common_init(&gtwx5715_pci);

	return 0;
}

subsys_initcall(gtwx5715_pci_init);
