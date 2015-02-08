

#include <linux/init.h>
#include <linux/of_platform.h>
#include <linux/rtc.h>

#include <asm/machdep.h>
#include <asm/prom.h>
#include <asm/udbg.h>
#include <asm/time.h>
#include <asm/uic.h>
#include <asm/pci-bridge.h>
#include <asm/ppc4xx.h>

static __initdata struct of_device_id walnut_of_bus[] = {
	{ .compatible = "ibm,plb3", },
	{ .compatible = "ibm,opb", },
	{ .compatible = "ibm,ebc", },
	{},
};

static int __init walnut_device_probe(void)
{
	of_platform_bus_probe(NULL, walnut_of_bus, NULL);
	of_instantiate_rtc();

	return 0;
}
machine_device_initcall(walnut, walnut_device_probe);

static int __init walnut_probe(void)
{
	unsigned long root = of_get_flat_dt_root();

	if (!of_flat_dt_is_compatible(root, "ibm,walnut"))
		return 0;

	ppc_pci_flags = PPC_PCI_REASSIGN_ALL_RSRC;

	return 1;
}

define_machine(walnut) {
	.name			= "Walnut",
	.probe			= walnut_probe,
	.progress		= udbg_progress,
	.init_IRQ		= uic_init_tree,
	.get_irq		= uic_get_irq,
	.restart		= ppc4xx_reset_system,
	.calibrate_decr		= generic_calibrate_decr,
};
