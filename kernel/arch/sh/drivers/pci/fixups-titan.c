
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <mach/titan.h>
#include "pci-sh4.h"

static char titan_irq_tab[] __initdata = {
	TITAN_IRQ_WAN,
	TITAN_IRQ_LAN,
	TITAN_IRQ_MPCIA,
	TITAN_IRQ_MPCIB,
	TITAN_IRQ_USB,
};

int __init pcibios_map_platform_irq(struct pci_dev *pdev, u8 slot, u8 pin)
{
	int irq = titan_irq_tab[slot];

	printk("PCI: Mapping TITAN IRQ for slot %d, pin %c to irq %d\n",
		slot, pin - 1 + 'A', irq);

	return irq;
}
