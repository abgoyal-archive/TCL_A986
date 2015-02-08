

#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/pci.h>

void __init titan_ht_pcibios_fixup_bus(struct pci_bus *bus)
{
        struct pci_bus *current_bus = bus;
        struct pci_dev *devices;
        struct list_head *devices_link;

	list_for_each(devices_link, &(current_bus->devices)) {
                devices = pci_dev_b(devices_link);
                if (devices == NULL)
                        continue;
	}

	/*
	 * PLX and SPKT related changes go here
	 */

}
