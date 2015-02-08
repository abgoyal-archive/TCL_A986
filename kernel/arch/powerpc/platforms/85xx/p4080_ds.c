

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/kdev_t.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include <asm/system.h>
#include <asm/time.h>
#include <asm/machdep.h>
#include <asm/pci-bridge.h>
#include <mm/mmu_decl.h>
#include <asm/prom.h>
#include <asm/udbg.h>
#include <asm/mpic.h>

#include <linux/of_platform.h>
#include <sysdev/fsl_soc.h>
#include <sysdev/fsl_pci.h>

#include "corenet_ds.h"

#ifdef CONFIG_PCI
static int primary_phb_addr;
#endif

static int __init p4080_ds_probe(void)
{
	unsigned long root = of_get_flat_dt_root();

	if (of_flat_dt_is_compatible(root, "fsl,P4080DS")) {
#ifdef CONFIG_PCI
		/* treat PCIe1 as primary,
		 * shouldn't matter as we have no ISA on the board
		 */
		primary_phb_addr = 0x0000;
#endif
		return 1;
	} else {
		return 0;
	}
}

define_machine(p4080_ds) {
	.name			= "P4080 DS",
	.probe			= p4080_ds_probe,
	.setup_arch		= corenet_ds_setup_arch,
	.init_IRQ		= corenet_ds_pic_init,
#ifdef CONFIG_PCI
	.pcibios_fixup_bus	= fsl_pcibios_fixup_bus,
#endif
	.get_irq		= mpic_get_coreint_irq,
	.restart		= fsl_rstcr_restart,
	.calibrate_decr		= generic_calibrate_decr,
	.progress		= udbg_progress,
};

machine_device_initcall(p4080_ds, corenet_ds_publish_devices);
machine_arch_initcall(p4080_ds, swiotlb_setup_bus_notifier);
