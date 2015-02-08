

#include <linux/pci.h>
#include <linux/of_platform.h>

#include <asm/time.h>
#include <asm/ipic.h>
#include <asm/udbg.h>

#include "mpc83xx.h"

static void __init asp834x_setup_arch(void)
{
	if (ppc_md.progress)
		ppc_md.progress("asp834x_setup_arch()", 0);

	mpc834x_usb_cfg();
}

static void __init asp834x_init_IRQ(void)
{
	struct device_node *np;

	np = of_find_node_by_type(NULL, "ipic");
	if (!np)
		return;

	ipic_init(np, 0);

	of_node_put(np);

	/* Initialize the default interrupt mapping priorities,
	 * in case the boot rom changed something on us.
	 */
	ipic_set_default_priority();
}

static struct __initdata of_device_id asp8347_ids[] = {
	{ .type = "soc", },
	{ .compatible = "soc", },
	{ .compatible = "simple-bus", },
	{ .compatible = "gianfar", },
	{},
};

static int __init asp8347_declare_of_platform_devices(void)
{
	of_platform_bus_probe(NULL, asp8347_ids, NULL);
	return 0;
}
machine_device_initcall(asp834x, asp8347_declare_of_platform_devices);

static int __init asp834x_probe(void)
{
	unsigned long root = of_get_flat_dt_root();
	return of_flat_dt_is_compatible(root, "analogue-and-micro,asp8347e");
}

define_machine(asp834x) {
	.name			= "ASP8347E",
	.probe			= asp834x_probe,
	.setup_arch		= asp834x_setup_arch,
	.init_IRQ		= asp834x_init_IRQ,
	.get_irq		= ipic_get_irq,
	.restart		= mpc83xx_restart,
	.time_init		= mpc83xx_time_init,
	.calibrate_decr		= generic_calibrate_decr,
	.progress		= udbg_progress,
};
