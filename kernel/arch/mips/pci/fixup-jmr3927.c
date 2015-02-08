
#include <linux/types.h>
#include <asm/txx9/pci.h>
#include <asm/txx9/jmr3927.h>

int __init jmr3927_pci_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	unsigned char irq = pin;

	/* IRQ rotation (PICMG) */
	irq--;			/* 0-3 */
	if (slot == TX3927_PCIC_IDSEL_AD_TO_SLOT(23)) {
		/* PCI CardSlot (IDSEL=A23, DevNu=12) */
		/* PCIA => PCIC (IDSEL=A23) */
		/* NOTE: JMR3927 JP1 must be set to OPEN */
		irq = (irq + 2) % 4;
	} else if (slot == TX3927_PCIC_IDSEL_AD_TO_SLOT(22)) {
		/* PCI CardSlot (IDSEL=A22, DevNu=11) */
		/* PCIA => PCIA (IDSEL=A22) */
		/* NOTE: JMR3927 JP1 must be set to OPEN */
		irq = (irq + 0) % 4;
	} else {
		/* PCI Backplane */
		if (txx9_pci_option & TXX9_PCI_OPT_PICMG)
			irq = (irq + 33 - slot) % 4;
		else
			irq = (irq + 3 + slot) % 4;
	}
	irq++;			/* 1-4 */

	switch (irq) {
	case 1:
		irq = JMR3927_IRQ_IOC_PCIA;
		break;
	case 2:
		irq = JMR3927_IRQ_IOC_PCIB;
		break;
	case 3:
		irq = JMR3927_IRQ_IOC_PCIC;
		break;
	case 4:
		irq = JMR3927_IRQ_IOC_PCID;
		break;
	}

	/* Check OnBoard Ethernet (IDSEL=A24, DevNu=13) */
	if (dev->bus->parent == NULL &&
	    slot == TX3927_PCIC_IDSEL_AD_TO_SLOT(24))
		irq = JMR3927_IRQ_ETHER0;
	return irq;
}
