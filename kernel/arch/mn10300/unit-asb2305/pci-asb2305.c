
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include "pci-asb2305.h"

resource_size_t pcibios_align_resource(void *data, const struct resource *res,
				resource_size_t size, resource_size_t align)
{
	resource_size_t start = res->start;

#if 0
	struct pci_dev *dev = data;

	printk(KERN_DEBUG
	       "### PCIBIOS_ALIGN_RESOURCE(%s,,{%08lx-%08lx,%08lx},%lx)\n",
	       pci_name(dev),
	       res->start,
	       res->end,
	       res->flags,
	       size
	       );
#endif

	if ((res->flags & IORESOURCE_IO) && (start & 0x300))
		start = (start + 0x3ff) & ~0x3ff;

	return start;
}


static void __init pcibios_allocate_bus_resources(struct list_head *bus_list)
{
	struct pci_bus *bus;
	struct pci_dev *dev;
	int idx;
	struct resource *r, *pr;

	/* Depth-First Search on bus tree */
	list_for_each_entry(bus, bus_list, node) {
		dev = bus->self;
		if (dev) {
			for (idx = PCI_BRIDGE_RESOURCES;
			     idx < PCI_NUM_RESOURCES;
			     idx++) {
				r = &dev->resource[idx];
				if (!r->flags)
					continue;
				pr = pci_find_parent_resource(dev, r);
				if (!r->start ||
				    !pr ||
				    request_resource(pr, r) < 0) {
					printk(KERN_ERR "PCI:"
					       " Cannot allocate resource"
					       " region %d of bridge %s\n",
					       idx, pci_name(dev));
					/* Something is wrong with the region.
					 * Invalidate the resource to prevent
					 * child resource allocations in this
					 * range. */
					r->start = r->end = 0;
					r->flags = 0;
				}
			}
		}
		pcibios_allocate_bus_resources(&bus->children);
	}
}

static void __init pcibios_allocate_resources(int pass)
{
	struct pci_dev *dev = NULL;
	int idx, disabled;
	u16 command;
	struct resource *r, *pr;

	for_each_pci_dev(dev) {
		pci_read_config_word(dev, PCI_COMMAND, &command);
		for (idx = 0; idx < 6; idx++) {
			r = &dev->resource[idx];
			if (r->parent)		/* Already allocated */
				continue;
			if (!r->start)		/* Address not assigned */
				continue;
			if (r->flags & IORESOURCE_IO)
				disabled = !(command & PCI_COMMAND_IO);
			else
				disabled = !(command & PCI_COMMAND_MEMORY);
			if (pass == disabled) {
				DBG("PCI[%s]: Resource %08lx-%08lx"
				    " (f=%lx, d=%d, p=%d)\n",
				    pci_name(dev), r->start, r->end, r->flags,
				    disabled, pass);
				pr = pci_find_parent_resource(dev, r);
				if (!pr || request_resource(pr, r) < 0) {
					printk(KERN_ERR "PCI:"
					       " Cannot allocate resource"
					       " region %d of device %s\n",
					       idx, pci_name(dev));
					/* We'll assign a new address later */
					r->end -= r->start;
					r->start = 0;
				}
			}
		}
		if (!pass) {
			r = &dev->resource[PCI_ROM_RESOURCE];
			if (r->flags & IORESOURCE_ROM_ENABLE) {
				/* Turn the ROM off, leave the resource region,
				 * but keep it unregistered. */
				u32 reg;
				DBG("PCI: Switching off ROM of %s\n",
				    pci_name(dev));
				r->flags &= ~IORESOURCE_ROM_ENABLE;
				pci_read_config_dword(
					dev, dev->rom_base_reg, &reg);
				pci_write_config_dword(
					dev, dev->rom_base_reg,
					reg & ~PCI_ROM_ADDRESS_ENABLE);
			}
		}
	}
}

static int __init pcibios_assign_resources(void)
{
	struct pci_dev *dev = NULL;
	struct resource *r, *pr;

	if (!(pci_probe & PCI_ASSIGN_ROMS)) {
		/* Try to use BIOS settings for ROMs, otherwise let
		   pci_assign_unassigned_resources() allocate the new
		   addresses. */
		for_each_pci_dev(dev) {
			r = &dev->resource[PCI_ROM_RESOURCE];
			if (!r->flags || !r->start)
				continue;
			pr = pci_find_parent_resource(dev, r);
			if (!pr || request_resource(pr, r) < 0) {
				r->end -= r->start;
				r->start = 0;
			}
		}
	}

	pci_assign_unassigned_resources();

	return 0;
}

fs_initcall(pcibios_assign_resources);

void __init pcibios_resource_survey(void)
{
	DBG("PCI: Allocating resources\n");
	pcibios_allocate_bus_resources(&pci_root_buses);
	pcibios_allocate_resources(0);
	pcibios_allocate_resources(1);
}

unsigned int pcibios_max_latency = 255;

void pcibios_set_master(struct pci_dev *dev)
{
	u8 lat;

	pci_read_config_byte(dev, PCI_LATENCY_TIMER, &lat);

	if (lat < 16)
		lat = (64 <= pcibios_max_latency) ? 64 : pcibios_max_latency;
	else if (lat > pcibios_max_latency)
		lat = pcibios_max_latency;
	else
		return;

	pci_write_config_byte(dev, PCI_LATENCY_TIMER, lat);
}

int pci_mmap_page_range(struct pci_dev *dev, struct vm_area_struct *vma,
			enum pci_mmap_state mmap_state, int write_combine)
{
	unsigned long prot;

	/* Leave vm_pgoff as-is, the PCI space address is the physical
	 * address on this platform.
	 */
	vma->vm_flags |= VM_LOCKED | VM_IO;

	prot = pgprot_val(vma->vm_page_prot);
	prot &= ~_PAGE_CACHE;
	vma->vm_page_prot = __pgprot(prot);

	/* Write-combine setting is ignored */
	if (io_remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}
