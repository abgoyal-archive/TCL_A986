

#include <linux/module.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/ptrace.h>
#include <linux/mman.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/stddef.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/hardirq.h>
#include <linux/gfp.h>

#include <asm/pgalloc.h>
#include <asm/io.h>
#include <asm/mmu_context.h>
#include <asm/pgtable.h>
#include <asm/mmu.h>
#include <asm/uaccess.h>
#include <asm/smp.h>

static int map_page(unsigned long va, unsigned long pa, pgprot_t prot)
{
	pgd_t *pge;
	pud_t *pue;
	pmd_t *pme;
	pte_t *pte;
	int err = -ENOMEM;

	/* Use upper 10 bits of VA to index the first level map */
	pge = pgd_offset_k(va);
	pue = pud_offset(pge, va);
	pme = pmd_offset(pue, va);

	/* Use middle 10 bits of VA to index the second-level map */
	pte = pte_alloc_kernel(pme, va);
	if (pte != 0) {
		err = 0;
		set_pte(pte, mk_pte_phys(pa & PAGE_MASK, prot));
	}

	return err;
}

void *consistent_alloc(gfp_t gfp, size_t size, dma_addr_t *dma_handle)
{
	struct vm_struct *area;
	unsigned long page, va, pa;
	void *ret;
	int order, err, i;

	if (in_interrupt())
		BUG();

	/* only allocate page size areas */
	size = PAGE_ALIGN(size);
	order = get_order(size);

	page = __get_free_pages(gfp, order);
	if (!page) {
		BUG();
		return NULL;
	}

	/* allocate some common virtual space to map the new pages */
	area = get_vm_area(size, VM_ALLOC);
	if (area == 0) {
		free_pages(page, order);
		return NULL;
	}
	va = VMALLOC_VMADDR(area->addr);
	ret = (void *) va;

	/* this gives us the real physical address of the first page */
	*dma_handle = pa = virt_to_bus((void *) page);

	/* set refcount=1 on all pages in an order>0 allocation so that vfree() will actually free
	 * all pages that were allocated.
	 */
	if (order > 0) {
		struct page *rpage = virt_to_page(page);
		split_page(rpage, order);
	}

	err = 0;
	for (i = 0; i < size && err == 0; i += PAGE_SIZE)
		err = map_page(va + i, pa + i, PAGE_KERNEL_NOCACHE);

	if (err) {
		vfree((void *) va);
		return NULL;
	}

	/* we need to ensure that there are no cachelines in use, or worse dirty in this area
	 * - can't do until after virtual address mappings are created
	 */
	frv_cache_invalidate(va, va + size);

	return ret;
}

void consistent_free(void *vaddr)
{
	if (in_interrupt())
		BUG();
	vfree(vaddr);
}

void consistent_sync(void *vaddr, size_t size, int direction)
{
	unsigned long start = (unsigned long) vaddr;
	unsigned long end   = start + size;

	switch (direction) {
	case PCI_DMA_NONE:
		BUG();
	case PCI_DMA_FROMDEVICE:	/* invalidate only */
		frv_cache_invalidate(start, end);
		break;
	case PCI_DMA_TODEVICE:		/* writeback only */
		frv_dcache_writeback(start, end);
		break;
	case PCI_DMA_BIDIRECTIONAL:	/* writeback and invalidate */
		frv_dcache_writeback(start, end);
		break;
	}
}


void consistent_sync_page(struct page *page, unsigned long offset,
			  size_t size, int direction)
{
	void *start;

	start = page_address(page) + offset;
	consistent_sync(start, size, direction);
}
