

#include <linux/types.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/pci.h>
#include <linux/gfp.h>
#include <asm/io.h>
#include <asm/cacheflush.h>


void *
dma_alloc_coherent(struct device *dev,size_t size,dma_addr_t *handle,gfp_t flag)
{
	unsigned long ret;
	unsigned long uncached = 0;

	/* ignore region speicifiers */

	flag &= ~(__GFP_DMA | __GFP_HIGHMEM);

	if (dev == NULL || (dev->coherent_dma_mask < 0xffffffff))
		flag |= GFP_DMA;
	ret = (unsigned long)__get_free_pages(flag, get_order(size));

	if (ret == 0)
		return NULL;

	/* We currently don't support coherent memory outside KSEG */

	if (ret < XCHAL_KSEG_CACHED_VADDR
	    || ret >= XCHAL_KSEG_CACHED_VADDR + XCHAL_KSEG_SIZE)
		BUG();


	if (ret != 0) {
		memset((void*) ret, 0, size);
		uncached = ret+XCHAL_KSEG_BYPASS_VADDR-XCHAL_KSEG_CACHED_VADDR;
		*handle = virt_to_bus((void*)ret);
		__flush_invalidate_dcache_range(ret, size);
	}

	return (void*)uncached;
}

void dma_free_coherent(struct device *hwdev, size_t size,
			 void *vaddr, dma_addr_t dma_handle)
{
	long addr=(long)vaddr+XCHAL_KSEG_CACHED_VADDR-XCHAL_KSEG_BYPASS_VADDR;

	if (addr < 0 || addr >= XCHAL_KSEG_SIZE)
		BUG();

	free_pages(addr, get_order(size));
}


void consistent_sync(void *vaddr, size_t size, int direction)
{
	switch (direction) {
	case PCI_DMA_NONE:
		BUG();
	case PCI_DMA_FROMDEVICE:        /* invalidate only */
		__invalidate_dcache_range((unsigned long)vaddr,
				          (unsigned long)size);
		break;

	case PCI_DMA_TODEVICE:          /* writeback only */
	case PCI_DMA_BIDIRECTIONAL:     /* writeback and invalidate */
		__flush_invalidate_dcache_range((unsigned long)vaddr,
				    		(unsigned long)size);
		break;
	}
}
