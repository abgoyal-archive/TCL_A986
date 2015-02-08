

#include <linux/highmem.h>
#include <linux/module.h>

void *kmap_atomic_prot(struct page *page, enum km_type type, pgprot_t prot)
{
	unsigned int idx;
	unsigned long vaddr;

	/* even !CONFIG_PREEMPT needs this, for in_atomic in do_page_fault */
	pagefault_disable();
	if (!PageHighMem(page))
		return page_address(page);

	debug_kmap_atomic(type);
	idx = type + KM_TYPE_NR*smp_processor_id();
	vaddr = __fix_to_virt(FIX_KMAP_BEGIN + idx);
#ifdef CONFIG_DEBUG_HIGHMEM
	BUG_ON(!pte_none(*(kmap_pte-idx)));
#endif
	__set_pte_at(&init_mm, vaddr, kmap_pte-idx, mk_pte(page, prot), 1);
	local_flush_tlb_page(NULL, vaddr);

	return (void*) vaddr;
}
EXPORT_SYMBOL(kmap_atomic_prot);

void kunmap_atomic(void *kvaddr, enum km_type type)
{
#ifdef CONFIG_DEBUG_HIGHMEM
	unsigned long vaddr = (unsigned long) kvaddr & PAGE_MASK;
	enum fixed_addresses idx = type + KM_TYPE_NR*smp_processor_id();

	if (vaddr < __fix_to_virt(FIX_KMAP_END)) {
		pagefault_enable();
		return;
	}

	BUG_ON(vaddr != __fix_to_virt(FIX_KMAP_BEGIN + idx));

	/*
	 * force other mappings to Oops if they'll try to access
	 * this pte without first remap it
	 */
	pte_clear(&init_mm, vaddr, kmap_pte-idx);
	local_flush_tlb_page(NULL, vaddr);
#endif
	pagefault_enable();
}
EXPORT_SYMBOL(kunmap_atomic);
