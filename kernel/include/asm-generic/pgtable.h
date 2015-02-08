
#ifndef _ASM_GENERIC_PGTABLE_H
#define _ASM_GENERIC_PGTABLE_H

#ifndef __ASSEMBLY__
#ifdef CONFIG_MMU

#ifndef __HAVE_ARCH_PTEP_SET_ACCESS_FLAGS
#define ptep_set_access_flags(__vma, __address, __ptep, __entry, __dirty) \
({									  \
	int __changed = !pte_same(*(__ptep), __entry);			  \
	if (__changed) {						  \
		set_pte_at((__vma)->vm_mm, (__address), __ptep, __entry); \
		flush_tlb_page(__vma, __address);			  \
	}								  \
	__changed;							  \
})
#endif

#ifndef __HAVE_ARCH_PTEP_TEST_AND_CLEAR_YOUNG
#define ptep_test_and_clear_young(__vma, __address, __ptep)		\
({									\
	pte_t __pte = *(__ptep);					\
	int r = 1;							\
	if (!pte_young(__pte))						\
		r = 0;							\
	else								\
		set_pte_at((__vma)->vm_mm, (__address),			\
			   (__ptep), pte_mkold(__pte));			\
	r;								\
})
#endif

#ifndef __HAVE_ARCH_PTEP_CLEAR_YOUNG_FLUSH
#define ptep_clear_flush_young(__vma, __address, __ptep)		\
({									\
	int __young;							\
	__young = ptep_test_and_clear_young(__vma, __address, __ptep);	\
	if (__young)							\
		flush_tlb_page(__vma, __address);			\
	__young;							\
})
#endif

#ifndef __HAVE_ARCH_PTEP_GET_AND_CLEAR
#define ptep_get_and_clear(__mm, __address, __ptep)			\
({									\
	pte_t __pte = *(__ptep);					\
	pte_clear((__mm), (__address), (__ptep));			\
	__pte;								\
})
#endif

#ifndef __HAVE_ARCH_PTEP_GET_AND_CLEAR_FULL
#define ptep_get_and_clear_full(__mm, __address, __ptep, __full)	\
({									\
	pte_t __pte;							\
	__pte = ptep_get_and_clear((__mm), (__address), (__ptep));	\
	__pte;								\
})
#endif

#ifndef __HAVE_ARCH_PTE_CLEAR_NOT_PRESENT_FULL
#define pte_clear_not_present_full(__mm, __address, __ptep, __full)	\
do {									\
	pte_clear((__mm), (__address), (__ptep));			\
} while (0)
#endif

#ifndef __HAVE_ARCH_PTEP_CLEAR_FLUSH
#define ptep_clear_flush(__vma, __address, __ptep)			\
({									\
	pte_t __pte;							\
	__pte = ptep_get_and_clear((__vma)->vm_mm, __address, __ptep);	\
	flush_tlb_page(__vma, __address);				\
	__pte;								\
})
#endif

#ifndef __HAVE_ARCH_PTEP_SET_WRPROTECT
struct mm_struct;
static inline void ptep_set_wrprotect(struct mm_struct *mm, unsigned long address, pte_t *ptep)
{
	pte_t old_pte = *ptep;
	set_pte_at(mm, address, ptep, pte_wrprotect(old_pte));
}
#endif

#ifndef __HAVE_ARCH_PTE_SAME
#define pte_same(A,B)	(pte_val(A) == pte_val(B))
#endif

#ifndef __HAVE_ARCH_PAGE_TEST_DIRTY
#define page_test_dirty(page)		(0)
#endif

#ifndef __HAVE_ARCH_PAGE_CLEAR_DIRTY
#define page_clear_dirty(page)		do { } while (0)
#endif

#ifndef __HAVE_ARCH_PAGE_TEST_DIRTY
#define pte_maybe_dirty(pte)		pte_dirty(pte)
#else
#define pte_maybe_dirty(pte)		(1)
#endif

#ifndef __HAVE_ARCH_PAGE_TEST_AND_CLEAR_YOUNG
#define page_test_and_clear_young(page) (0)
#endif

#ifndef __HAVE_ARCH_PGD_OFFSET_GATE
#define pgd_offset_gate(mm, addr)	pgd_offset(mm, addr)
#endif

#ifndef __HAVE_ARCH_MOVE_PTE
#define move_pte(pte, prot, old_addr, new_addr)	(pte)
#endif

#ifndef pgprot_noncached
#define pgprot_noncached(prot)	(prot)
#endif

#ifndef pgprot_writecombine
#define pgprot_writecombine pgprot_noncached
#endif


#define pgd_addr_end(addr, end)						\
({	unsigned long __boundary = ((addr) + PGDIR_SIZE) & PGDIR_MASK;	\
	(__boundary - 1 < (end) - 1)? __boundary: (end);		\
})

#ifndef pud_addr_end
#define pud_addr_end(addr, end)						\
({	unsigned long __boundary = ((addr) + PUD_SIZE) & PUD_MASK;	\
	(__boundary - 1 < (end) - 1)? __boundary: (end);		\
})
#endif

#ifndef pmd_addr_end
#define pmd_addr_end(addr, end)						\
({	unsigned long __boundary = ((addr) + PMD_SIZE) & PMD_MASK;	\
	(__boundary - 1 < (end) - 1)? __boundary: (end);		\
})
#endif

void pgd_clear_bad(pgd_t *);
void pud_clear_bad(pud_t *);
void pmd_clear_bad(pmd_t *);

static inline int pgd_none_or_clear_bad(pgd_t *pgd)
{
	if (pgd_none(*pgd))
		return 1;
	if (unlikely(pgd_bad(*pgd))) {
		pgd_clear_bad(pgd);
		return 1;
	}
	return 0;
}

static inline int pud_none_or_clear_bad(pud_t *pud)
{
	if (pud_none(*pud))
		return 1;
	if (unlikely(pud_bad(*pud))) {
		pud_clear_bad(pud);
		return 1;
	}
	return 0;
}

static inline int pmd_none_or_clear_bad(pmd_t *pmd)
{
	if (pmd_none(*pmd))
		return 1;
	if (unlikely(pmd_bad(*pmd))) {
		pmd_clear_bad(pmd);
		return 1;
	}
	return 0;
}

static inline pte_t __ptep_modify_prot_start(struct mm_struct *mm,
					     unsigned long addr,
					     pte_t *ptep)
{
	/*
	 * Get the current pte state, but zero it out to make it
	 * non-present, preventing the hardware from asynchronously
	 * updating it.
	 */
	return ptep_get_and_clear(mm, addr, ptep);
}

static inline void __ptep_modify_prot_commit(struct mm_struct *mm,
					     unsigned long addr,
					     pte_t *ptep, pte_t pte)
{
	/*
	 * The pte is non-present, so there's no hardware state to
	 * preserve.
	 */
	set_pte_at(mm, addr, ptep, pte);
}

#ifndef __HAVE_ARCH_PTEP_MODIFY_PROT_TRANSACTION
static inline pte_t ptep_modify_prot_start(struct mm_struct *mm,
					   unsigned long addr,
					   pte_t *ptep)
{
	return __ptep_modify_prot_start(mm, addr, ptep);
}

static inline void ptep_modify_prot_commit(struct mm_struct *mm,
					   unsigned long addr,
					   pte_t *ptep, pte_t pte)
{
	__ptep_modify_prot_commit(mm, addr, ptep, pte);
}
#endif /* __HAVE_ARCH_PTEP_MODIFY_PROT_TRANSACTION */
#endif /* CONFIG_MMU */

#ifndef __HAVE_ARCH_ENTER_LAZY_MMU_MODE
#define arch_enter_lazy_mmu_mode()	do {} while (0)
#define arch_leave_lazy_mmu_mode()	do {} while (0)
#define arch_flush_lazy_mmu_mode()	do {} while (0)
#endif

#ifndef __HAVE_ARCH_START_CONTEXT_SWITCH
#define arch_start_context_switch(prev)	do {} while (0)
#endif

#ifndef __HAVE_PFNMAP_TRACKING
static inline int track_pfn_vma_new(struct vm_area_struct *vma, pgprot_t *prot,
					unsigned long pfn, unsigned long size)
{
	return 0;
}

static inline int track_pfn_vma_copy(struct vm_area_struct *vma)
{
	return 0;
}

static inline void untrack_pfn_vma(struct vm_area_struct *vma,
					unsigned long pfn, unsigned long size)
{
}
#else
extern int track_pfn_vma_new(struct vm_area_struct *vma, pgprot_t *prot,
				unsigned long pfn, unsigned long size);
extern int track_pfn_vma_copy(struct vm_area_struct *vma);
extern void untrack_pfn_vma(struct vm_area_struct *vma, unsigned long pfn,
				unsigned long size);
#endif

#endif /* !__ASSEMBLY__ */

#endif /* _ASM_GENERIC_PGTABLE_H */
