
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/mach-pnx8550/pci.h>
#include <asm/mach-pnx8550/int.h>


#undef	DEBUG
#ifdef 	DEBUG
#define	DBG(x...)	printk(x)
#else
#define	DBG(x...)
#endif

extern char pnx8550_irq_tab[][5];

void __init pcibios_fixup_resources(struct pci_dev *dev)
{
	/* no need to fixup IO resources */
}

void __init pcibios_fixup(void)
{
	/* nothing to do here */
}

int __init pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	return pnx8550_irq_tab[slot][pin];
}

/* Do platform specific device initialization at pci_enable_device() time */
int pcibios_plat_dev_init(struct pci_dev *dev)
{
	return 0;
}
