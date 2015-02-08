

#include <asm/machdep.h>
#include <asm/pci-bridge.h>
#include <asm/ppc4xx.h>
#include <asm/prom.h>
#include <asm/time.h>
#include <asm/udbg.h>
#include <asm/uic.h>

#include <linux/init.h>
#include <linux/of_platform.h>

static __initdata struct of_device_id ppc44x_of_bus[] = {
	{ .compatible = "ibm,plb4", },
	{ .compatible = "ibm,opb", },
	{ .compatible = "ibm,ebc", },
	{ .compatible = "simple-bus", },
	{},
};

static int __init ppc44x_device_probe(void)
{
	of_platform_bus_probe(NULL, ppc44x_of_bus, NULL);

	return 0;
}
machine_device_initcall(ppc44x_simple, ppc44x_device_probe);

static char *board[] __initdata = {
	"amcc,arches",
	"amcc,bamboo",
	"amcc,canyonlands",
	"amcc,glacier",
	"ibm,ebony",
	"amcc,eiger",
	"amcc,katmai",
	"amcc,rainier",
	"amcc,redwood",
	"amcc,sequoia",
	"amcc,taishan",
	"amcc,yosemite",
	"mosaixtech,icon"
};

static int __init ppc44x_probe(void)
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

define_machine(ppc44x_simple) {
	.name = "PowerPC 44x Platform",
	.probe = ppc44x_probe,
	.progress = udbg_progress,
	.init_IRQ = uic_init_tree,
	.get_irq = uic_get_irq,
	.restart = ppc4xx_reset_system,
	.calibrate_decr = generic_calibrate_decr,
};
