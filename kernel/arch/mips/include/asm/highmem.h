
#ifndef _ASM_HIGHMEM_H
#define _ASM_HIGHMEM_H

#ifdef __KERNEL__

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <asm/kmap_types.h>

/* undef for production */
#define HIGHMEM_DEBUG 1

/* declarations for highmem.c */
extern unsigned long highstart_pfn, highend_pfn;

extern pte_t *pkmap_page_table;

#define LAST_PKMAP 1024
#define LAST_PKMAP_MASK (LAST_PKMAP-1)
#define PKMAP_NR(virt)  ((virt-PKMAP_BASE) >> PAGE_SHIFT)
#define PKMAP_ADDR(nr)  (PKMAP_BASE + ((nr) << PAGE_SHIFT))

extern void * kmap_high(struct page *page);
extern void kunmap_high(struct page *page);

extern void *__kmap(struct page *page);
extern void __kunmap(struct page *page);
extern void *__kmap_atomic(struct page *page, enum km_type type);
extern void __kunmap_atomic(void *kvaddr, enum km_type type);
extern void *kmap_atomic_pfn(unsigned long pfn, enum km_type type);
extern struct page *__kmap_atomic_to_page(void *ptr);

#define kmap			__kmap
#define kunmap			__kunmap
#define kmap_atomic		__kmap_atomic
#define kunmap_atomic		__kunmap_atomic
#define kmap_atomic_to_page	__kmap_atomic_to_page

#define flush_cache_kmaps()	flush_cache_all()

extern void kmap_init(void);

#define kmap_prot PAGE_KERNEL

#endif /* __KERNEL__ */

#endif /* _ASM_HIGHMEM_H */
