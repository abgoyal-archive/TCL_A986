
/* drm_pci.h -- PCI DMA memory management wrappers for DRM -*- linux-c -*- */


#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include "drmP.h"

/**********************************************************************/
/** \name PCI memory */
/*@{*/

drm_dma_handle_t *drm_pci_alloc(struct drm_device * dev, size_t size, size_t align)
{
	drm_dma_handle_t *dmah;
#if 1
	unsigned long addr;
	size_t sz;
#endif

	/* pci_alloc_consistent only guarantees alignment to the smallest
	 * PAGE_SIZE order which is greater than or equal to the requested size.
	 * Return NULL here for now to make sure nobody tries for larger alignment
	 */
	if (align > size)
		return NULL;

	dmah = kmalloc(sizeof(drm_dma_handle_t), GFP_KERNEL);
	if (!dmah)
		return NULL;

	dmah->size = size;
	dmah->vaddr = dma_alloc_coherent(&dev->pdev->dev, size, &dmah->busaddr, GFP_KERNEL | __GFP_COMP);

	if (dmah->vaddr == NULL) {
		kfree(dmah);
		return NULL;
	}

	memset(dmah->vaddr, 0, size);

	/* XXX - Is virt_to_page() legal for consistent mem? */
	/* Reserve */
	for (addr = (unsigned long)dmah->vaddr, sz = size;
	     sz > 0; addr += PAGE_SIZE, sz -= PAGE_SIZE) {
		SetPageReserved(virt_to_page(addr));
	}

	return dmah;
}

EXPORT_SYMBOL(drm_pci_alloc);

void __drm_pci_free(struct drm_device * dev, drm_dma_handle_t * dmah)
{
#if 1
	unsigned long addr;
	size_t sz;
#endif

	if (dmah->vaddr) {
		/* XXX - Is virt_to_page() legal for consistent mem? */
		/* Unreserve */
		for (addr = (unsigned long)dmah->vaddr, sz = dmah->size;
		     sz > 0; addr += PAGE_SIZE, sz -= PAGE_SIZE) {
			ClearPageReserved(virt_to_page(addr));
		}
		dma_free_coherent(&dev->pdev->dev, dmah->size, dmah->vaddr,
				  dmah->busaddr);
	}
}

void drm_pci_free(struct drm_device * dev, drm_dma_handle_t * dmah)
{
	__drm_pci_free(dev, dmah);
	kfree(dmah);
}

EXPORT_SYMBOL(drm_pci_free);

/*@}*/
