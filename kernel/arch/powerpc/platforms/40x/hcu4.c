

#include <linux/init.h>
#include <linux/of_platform.h>

#include <asm/machdep.h>
#include <asm/prom.h>
#include <asm/udbg.h>
#include <asm/time.h>
#include <asm/uic.h>
#include <asm/ppc4xx.h>

static __initdata struct of_device_id hcu4_of_bus[] = {
	{ .compatible = "ibm,plb3", },
	{ .compatible = "ibm,opb", },
	{ .compatible = "ibm,ebc", },
	{},
};

static int __init hcu4_device_probe(void)
{
	of_platform_bus_probe(NULL, hcu4_of_bus, NULL);
	return 0;
}
machine_device_initcall(hcu4, hcu4_device_probe);

static int __init hcu4_probe(void)
{
	unsigned long root = of_get_flat_dt_root();

	if (!of_flat_dt_is_compatible(root, "netstal,hcu4"))
		return 0;

	return 1;
}

define_machine(hcu4) {
	.name			= "HCU4",
	.probe			= hcu4_probe,
	.progress		= udbg_progress,
	.init_IRQ		= uic_init_tree,
	.get_irq		= uic_get_irq,
	.restart		= ppc4xx_reset_system,
	.calibrate_decr		= generic_calibrate_decr,
};
