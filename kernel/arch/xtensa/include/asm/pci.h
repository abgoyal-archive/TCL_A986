

#ifndef _XTENSA_PCI_H
#define _XTENSA_PCI_H

#ifdef __KERNEL__


#define pcibios_assign_all_busses()	0

extern struct pci_controller* pcibios_alloc_controller(void);

static inline void pcibios_set_master(struct pci_dev *dev)
{
	/* No special bus mastering setup handling */
}

static inline void pcibios_penalize_isa_irq(int irq)
{
	/* We don't do dynamic PCI IRQ allocation */
}

/* Assume some values. (We should revise them, if necessary) */

#define PCIBIOS_MIN_IO		0x2000
#define PCIBIOS_MIN_MEM		0x10000000


#include <linux/types.h>
#include <linux/slab.h>
#include <asm/scatterlist.h>
#include <linux/string.h>
#include <asm/io.h>

struct pci_dev;


#define PCI_DMA_BUS_IS_PHYS	(1)

/* Map a range of PCI memory or I/O space for a device into user space */
int pci_mmap_page_range(struct pci_dev *pdev, struct vm_area_struct *vma,
                        enum pci_mmap_state mmap_state, int write_combine);

/* Tell drivers/pci/proc.c that we have pci_mmap_page_range() */
#define HAVE_PCI_MMAP	1

#endif /* __KERNEL__ */

/* Implement the pci_ DMA API in terms of the generic device dma_ one */
#include <asm-generic/pci-dma-compat.h>

/* Generic PCI */
#include <asm-generic/pci.h>

#endif	/* _XTENSA_PCI_H */
