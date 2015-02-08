
#ifndef _ASM_HIGHMEM_H
#define _ASM_HIGHMEM_H

#ifdef __KERNEL__

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/highmem.h>
#include <asm/kmap_types.h>
#include <asm/pgtable.h>

/* undef for production */
#undef HIGHMEM_DEBUG

/* declarations for highmem.c */
extern unsigned long highstart_pfn, highend_pfn;

extern pte_t *kmap_pte;
extern pgprot_t kmap_prot;
extern pte_t *pkmap_page_table;

extern void __init kmap_init(void);

#define PKMAP_BASE	0xfe000000UL
#define LAST_PKMAP	1024
#define LAST_PKMAP_MASK (LAST_PKMAP - 1)
#define PKMAP_NR(virt)  ((virt - PKMAP_BASE) >> PAGE_SHIFT)
#define PKMAP_ADDR(nr)  (PKMAP_BASE + ((nr) << PAGE_SHIFT))

extern unsigned long kmap_high(struct page *page);
extern void kunmap_high(struct page *page);

static inline unsigned long kmap(struct page *page)
{
	if (in_interrupt())
		BUG();
	if (page < highmem_start_page)
		return page_address(page);
	return kmap_high(page);
}

static inline void kunmap(struct page *page)
{
	if (in_interrupt())
		BUG();
	if (page < highmem_start_page)
		return;
	kunmap_high(page);
}

static inline unsigned long kmap_atomic(struct page *page, enum km_type type)
{
	enum fixed_addresses idx;
	unsigned long vaddr;

	if (page < highmem_start_page)
		return page_address(page);

	debug_kmap_atomic(type);
	idx = type + KM_TYPE_NR * smp_processor_id();
	vaddr = __fix_to_virt(FIX_KMAP_BEGIN + idx);
#if HIGHMEM_DEBUG
	if (!pte_none(*(kmap_pte - idx)))
		BUG();
#endif
	set_pte(kmap_pte - idx, mk_pte(page, kmap_prot));
	__flush_tlb_one(vaddr);

	return vaddr;
}

static inline void kunmap_atomic(unsigned long vaddr, enum km_type type)
{
#if HIGHMEM_DEBUG
	enum fixed_addresses idx = type + KM_TYPE_NR * smp_processor_id();

	if (vaddr < FIXADDR_START) /* FIXME */
		return;

	if (vaddr != __fix_to_virt(FIX_KMAP_BEGIN + idx))
		BUG();

	/*
	 * force other mappings to Oops if they'll try to access
	 * this pte without first remap it
	 */
	pte_clear(kmap_pte - idx);
	__flush_tlb_one(vaddr);
#endif
}

#endif /* __KERNEL__ */

#endif /* _ASM_HIGHMEM_H */
