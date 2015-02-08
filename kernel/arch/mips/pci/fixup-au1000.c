

#include <linux/pci.h>
#include <linux/init.h>

extern char irq_tab_alchemy[][5];

int __init pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	return irq_tab_alchemy[slot][pin];
}

/* Do platform specific device initialization at pci_enable_device() time */
int pcibios_plat_dev_init(struct pci_dev *dev)
{
	return 0;
}
