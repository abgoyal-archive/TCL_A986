

#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/mach-rc32434/rc32434.h>
#include <asm/mach-rc32434/irq.h>

static int __devinitdata irq_map[2][12] = {
	{0, 0, 2, 3, 2, 3, 0, 0, 0, 0, 0, 1},
	{0, 0, 1, 3, 0, 2, 1, 3, 0, 2, 1, 3}
};

int __devinit pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	int irq = 0;

	if (dev->bus->number < 2 && PCI_SLOT(dev->devfn) < 12)
		irq = irq_map[dev->bus->number][PCI_SLOT(dev->devfn)];

	return irq + GROUP4_IRQ_BASE + 4;
}

static void rc32434_pci_early_fixup(struct pci_dev *dev)
{
	if (PCI_SLOT(dev->devfn) == 6 && dev->bus->number == 0) {
		/* disable prefetched memory range */
		pci_write_config_word(dev, PCI_PREF_MEMORY_LIMIT, 0);
		pci_write_config_word(dev, PCI_PREF_MEMORY_BASE, 0x10);

		pci_write_config_byte(dev, PCI_CACHE_LINE_SIZE, 4);
	}
}

DECLARE_PCI_FIXUP_HEADER(PCI_ANY_ID, PCI_ANY_ID, rc32434_pci_early_fixup);

/* Do platform specific device initialization at pci_enable_device() time */
int pcibios_plat_dev_init(struct pci_dev *dev)
{
	return 0;
}
