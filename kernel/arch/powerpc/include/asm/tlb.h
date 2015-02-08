
#ifndef _ASM_POWERPC_TLB_H
#define _ASM_POWERPC_TLB_H
#ifdef __KERNEL__

#ifndef __powerpc64__
#include <asm/pgtable.h>
#endif
#include <asm/pgalloc.h>
#include <asm/tlbflush.h>
#ifndef __powerpc64__
#include <asm/page.h>
#include <asm/mmu.h>
#endif

#include <linux/pagemap.h>

#define tlb_start_vma(tlb, vma)	do { } while (0)
#define tlb_end_vma(tlb, vma)	do { } while (0)

extern void tlb_flush(struct mmu_gather *tlb);

/* Get the generic bits... */
#include <asm-generic/tlb.h>

extern void flush_hash_entry(struct mm_struct *mm, pte_t *ptep,
			     unsigned long address);

static inline void __tlb_remove_tlb_entry(struct mmu_gather *tlb, pte_t *ptep,
					  unsigned long address)
{
#ifdef CONFIG_PPC_STD_MMU_32
	if (pte_val(*ptep) & _PAGE_HASHPTE)
		flush_hash_entry(tlb->mm, ptep, address);
#endif
}

#endif /* __KERNEL__ */
#endif /* __ASM_POWERPC_TLB_H */
