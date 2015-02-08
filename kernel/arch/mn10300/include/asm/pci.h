
#ifndef _ASM_PCI_H
#define _ASM_PCI_H

#ifdef __KERNEL__
#include <linux/mm.h>		/* for struct page */

#if 0
#define __pcbdebug(FMT, ADDR, ...) \
	printk(KERN_DEBUG "PCIBRIDGE[%08x]: "FMT"\n", \
	       (u32)(ADDR), ##__VA_ARGS__)

#define __pcidebug(FMT, BUS, DEVFN, WHERE,...)		\
do {							\
	printk(KERN_DEBUG "PCI[%02x:%02x.%x + %02x]: "FMT"\n",	\
	       (BUS)->number,					\
	       PCI_SLOT(DEVFN),					\
	       PCI_FUNC(DEVFN),					\
	       (u32)(WHERE), ##__VA_ARGS__);			\
} while (0)

#else
#define __pcbdebug(FMT, ADDR, ...)		do {} while (0)
#define __pcidebug(FMT, BUS, DEVFN, WHERE, ...)	do {} while (0)
#endif


#ifdef CONFIG_PCI
#define pcibios_assign_all_busses()	1
extern void unit_pci_init(void);
#else
#define pcibios_assign_all_busses()	0
#endif

extern unsigned long pci_mem_start;
#define PCIBIOS_MIN_IO		0xBE000004
#define PCIBIOS_MIN_MEM		0xB8000000

void pcibios_set_master(struct pci_dev *dev);
void pcibios_penalize_isa_irq(int irq);


#include <linux/types.h>
#include <linux/slab.h>
#include <asm/scatterlist.h>
#include <linux/string.h>
#include <asm/io.h>

struct pci_dev;

#define PCI_DMA_BUS_IS_PHYS	(1)

/* Return the index of the PCI controller for device. */
static inline int pci_controller_num(struct pci_dev *dev)
{
	return 0;
}

#define HAVE_PCI_MMAP
extern int pci_mmap_page_range(struct pci_dev *dev, struct vm_area_struct *vma,
			       enum pci_mmap_state mmap_state,
			       int write_combine);

#endif /* __KERNEL__ */

/* implement the pci_ DMA API in terms of the generic device dma_ one */
#include <asm-generic/pci-dma-compat.h>

extern void pcibios_resource_to_bus(struct pci_dev *dev,
				    struct pci_bus_region *region,
				    struct resource *res);

extern void pcibios_bus_to_resource(struct pci_dev *dev,
				    struct resource *res,
				    struct pci_bus_region *region);

static inline struct resource *
pcibios_select_root(struct pci_dev *pdev, struct resource *res)
{
	struct resource *root = NULL;

	if (res->flags & IORESOURCE_IO)
		root = &ioport_resource;
	if (res->flags & IORESOURCE_MEM)
		root = &iomem_resource;

	return root;
}

static inline int pci_get_legacy_ide_irq(struct pci_dev *dev, int channel)
{
	return channel ? 15 : 14;
}

#endif /* _ASM_PCI_H */
