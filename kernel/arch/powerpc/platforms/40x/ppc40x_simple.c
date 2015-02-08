

#include <asm/machdep.h>
#include <asm/pci-bridge.h>
#include <asm/ppc4xx.h>
#include <asm/prom.h>
#include <asm/time.h>
#include <asm/udbg.h>
#include <asm/uic.h>

#include <linux/init.h>
#include <linux/of_platform.h>

static __initdata struct of_device_id ppc40x_of_bus[] = {
	{ .compatible = "ibm,plb3", },
	{ .compatible = "ibm,plb4", },
	{ .compatible = "ibm,opb", },
	{ .compatible = "ibm,ebc", },
	{ .compatible = "simple-bus", },
	{},
};

static int __init ppc40x_device_probe(void)
{
	of_platform_bus_probe(NULL, ppc40x_of_bus, NULL);

	return 0;
}
machine_device_initcall(ppc40x_simple, ppc40x_device_probe);

static char *board[] __initdata = {
	"amcc,acadia",
	"amcc,haleakala",
	"amcc,kilauea",
	"amcc,makalu",
	"est,hotfoot"
};

static int __init ppc40x_probe(void)
{
	unsigned long root = of_get_flat_dt_root();
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(board); i++) {
		if (of_flat_dt_is_compatible(root, board[i])) {
			ppc_pci_set_flags(PPC_PCI_REASSIGN_ALL_RSRC);
			return 1;
		}
	}

	return 0;
}

define_machine(ppc40x_simple) {
	.name = "PowerPC 40x Platform",
	.probe = ppc40x_probe,
	.progress = udbg_progress,
	.init_IRQ = uic_init_tree,
	.get_irq = uic_get_irq,
	.restart = ppc4xx_reset_system,
	.calibrate_decr = generic_calibrate_decr,
};
