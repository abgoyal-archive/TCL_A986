

#ifndef __PCI_OCTEON_H__
#define __PCI_OCTEON_H__

#include <linux/pci.h>

/* Some PCI cards require delays when accessing config space. */
#define PCI_CONFIG_SPACE_DELAY 10000

extern int (*octeon_pcibios_map_irq)(const struct pci_dev *dev,
				     u8 slot, u8 pin);

#define OCTEON_PCI_BAR1_HOLE_BITS 5
#define OCTEON_PCI_BAR1_HOLE_SIZE (1ul<<(OCTEON_PCI_BAR1_HOLE_BITS+3))

enum octeon_dma_bar_type {
	OCTEON_DMA_BAR_TYPE_INVALID,
	OCTEON_DMA_BAR_TYPE_SMALL,
	OCTEON_DMA_BAR_TYPE_BIG,
	OCTEON_DMA_BAR_TYPE_PCIE
};

extern enum octeon_dma_bar_type octeon_dma_bar_type;

#endif
