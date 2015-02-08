
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/io.h>
#include <asm/smp.h>
#include "pci-asb2305.h"

void __init pcibios_irq_init(void)
{
}

void __init pcibios_fixup_irqs(void)
{
	struct pci_dev *dev = NULL;
	u8 line, pin;

	while ((dev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, dev)) != NULL) {
		pci_read_config_byte(dev, PCI_INTERRUPT_PIN, &pin);
		if (pin) {
			dev->irq = XIRQ1;
			pci_write_config_byte(dev, PCI_INTERRUPT_LINE,
					      dev->irq);
		}
		pci_read_config_byte(dev, PCI_INTERRUPT_LINE, &line);
	}
}

void __init pcibios_penalize_isa_irq(int irq)
{
}

void pcibios_enable_irq(struct pci_dev *dev)
{
	pci_write_config_byte(dev, PCI_INTERRUPT_LINE, dev->irq);
}
