
#ifndef _ASM_HIGHMEM_H
#define _ASM_HIGHMEM_H

#ifdef __KERNEL__

#include <linux/interrupt.h>
#include <linux/threads.h>
#include <asm/kmap_types.h>
#include <asm/tlbflush.h>
#include <asm/paravirt.h>

/* declarations for highmem.c */
extern unsigned long highstart_pfn, highend_pfn;

extern pte_t *kmap_pte;
extern pgprot_t kmap_prot;
extern pte_t *pkmap_page_table;

#ifdef CONFIG_X86_PAE
#define LAST_PKMAP 512
#else
#define LAST_PKMAP 1024
#endif
#define PKMAP_BASE ( (FIXADDR_BOOT_START - PAGE_SIZE*(LAST_PKMAP + 1)) & PMD_MASK )
#define LAST_PKMAP_MASK (LAST_PKMAP-1)
#define PKMAP_NR(virt)  ((virt-PKMAP_BASE) >> PAGE_SHIFT)
#define PKMAP_ADDR(nr)  (PKMAP_BASE + ((nr) << PAGE_SHIFT))

extern void * FASTCALL(kmap_high(struct page *page));
extern void FASTCALL(kunmap_high(struct page *page));

void *kmap(struct page *page);
void kunmap(struct page *page);
void *kmap_atomic_prot(struct page *page, enum km_type type, pgprot_t prot);
void *kmap_atomic(struct page *page, enum km_type type);
void kunmap_atomic(void *kvaddr, enum km_type type);
void *kmap_atomic_pfn(unsigned long pfn, enum km_type type);
struct page *kmap_atomic_to_page(void *ptr);

#ifndef CONFIG_PARAVIRT
#define kmap_atomic_pte(page, type)	kmap_atomic(page, type)
#endif

#define flush_cache_kmaps()	do { } while (0)

#endif /* __KERNEL__ */

#endif /* _ASM_HIGHMEM_H */
