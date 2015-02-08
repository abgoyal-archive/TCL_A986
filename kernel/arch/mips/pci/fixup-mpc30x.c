
#include <linux/init.h>
#include <linux/pci.h>

#include <asm/vr41xx/mpc30x.h>

static const int internal_func_irqs[] __initdata = {
	VRC4173_CASCADE_IRQ,
	VRC4173_AC97_IRQ,
	VRC4173_USB_IRQ,
};

static const int irq_tab_mpc30x[] __initdata = {
 [12] = VRC4173_PCMCIA1_IRQ,
 [13] = VRC4173_PCMCIA2_IRQ,
 [29] = MQ200_IRQ,
};

int __init pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	if (slot == 30)
		return internal_func_irqs[PCI_FUNC(dev->devfn)];

	return irq_tab_mpc30x[slot];
}

/* Do platform specific device initialization at pci_enable_device() time */
int pcibios_plat_dev_init(struct pci_dev *dev)
{
	return 0;
}
