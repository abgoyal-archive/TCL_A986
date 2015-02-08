
#ifndef _ASM_GENERIC__TLB_H
#define _ASM_GENERIC__TLB_H

#include <linux/swap.h>
#include <asm/pgalloc.h>
#include <asm/tlbflush.h>

#ifdef CONFIG_SMP
  #ifdef ARCH_FREE_PTR_NR
    #define FREE_PTR_NR   ARCH_FREE_PTR_NR
  #else
    #define FREE_PTE_NR	506
  #endif
  #define tlb_fast_mode(tlb) ((tlb)->nr == ~0U)
#else
  #define FREE_PTE_NR	1
  #define tlb_fast_mode(tlb) 1
#endif

struct mmu_gather {
	struct mm_struct	*mm;
	unsigned int		nr;	/* set to ~0U means fast mode */
	unsigned int		need_flush;/* Really unmapped some ptes? */
	unsigned int		fullmm; /* non-zero means full mm flush */
	struct page *		pages[FREE_PTE_NR];
};

/* Users of the generic TLB shootdown code must declare this storage space. */
DECLARE_PER_CPU(struct mmu_gather, mmu_gathers);

static inline struct mmu_gather *
tlb_gather_mmu(struct mm_struct *mm, unsigned int full_mm_flush)
{
	struct mmu_gather *tlb = &get_cpu_var(mmu_gathers);

	tlb->mm = mm;

	/* Use fast mode if only one CPU is online */
	tlb->nr = num_online_cpus() > 1 ? 0U : ~0U;

	tlb->fullmm = full_mm_flush;

	return tlb;
}

static inline void
tlb_flush_mmu(struct mmu_gather *tlb, unsigned long start, unsigned long end)
{
	if (!tlb->need_flush)
		return;
	tlb->need_flush = 0;
	tlb_flush(tlb);
	if (!tlb_fast_mode(tlb)) {
		free_pages_and_swap_cache(tlb->pages, tlb->nr);
		tlb->nr = 0;
	}
}

static inline void
tlb_finish_mmu(struct mmu_gather *tlb, unsigned long start, unsigned long end)
{
	tlb_flush_mmu(tlb, start, end);

	/* keep the page table cache within bounds */
	check_pgt_cache();

	put_cpu_var(mmu_gathers);
}

static inline void tlb_remove_page(struct mmu_gather *tlb, struct page *page)
{
	tlb->need_flush = 1;
	if (tlb_fast_mode(tlb)) {
		free_page_and_swap_cache(page);
		return;
	}
	tlb->pages[tlb->nr++] = page;
	if (tlb->nr >= FREE_PTE_NR)
		tlb_flush_mmu(tlb, 0, 0);
}

#define tlb_remove_tlb_entry(tlb, ptep, address)		\
	do {							\
		tlb->need_flush = 1;				\
		__tlb_remove_tlb_entry(tlb, ptep, address);	\
	} while (0)

#define pte_free_tlb(tlb, ptep, address)			\
	do {							\
		tlb->need_flush = 1;				\
		__pte_free_tlb(tlb, ptep, address);		\
	} while (0)

#ifndef __ARCH_HAS_4LEVEL_HACK
#define pud_free_tlb(tlb, pudp, address)			\
	do {							\
		tlb->need_flush = 1;				\
		__pud_free_tlb(tlb, pudp, address);		\
	} while (0)
#endif

#define pmd_free_tlb(tlb, pmdp, address)			\
	do {							\
		tlb->need_flush = 1;				\
		__pmd_free_tlb(tlb, pmdp, address);		\
	} while (0)

#define tlb_migrate_finish(mm) do {} while (0)

#endif /* _ASM_GENERIC__TLB_H */
