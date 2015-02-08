

#ifndef _ASM_S390_PGTABLE_H
#define _ASM_S390_PGTABLE_H

#ifndef __ASSEMBLY__
#include <linux/sched.h>
#include <linux/mm_types.h>
#include <asm/bitops.h>
#include <asm/bug.h>
#include <asm/processor.h>

extern pgd_t swapper_pg_dir[] __attribute__ ((aligned (4096)));
extern void paging_init(void);
extern void vmem_map_init(void);

#define update_mmu_cache(vma, address, ptep)     do { } while (0)

extern char empty_zero_page[PAGE_SIZE];
#define ZERO_PAGE(vaddr) (virt_to_page(empty_zero_page))
#endif /* !__ASSEMBLY__ */

#ifndef __s390x__
# define PMD_SHIFT	20
# define PUD_SHIFT	20
# define PGDIR_SHIFT	20
#else /* __s390x__ */
# define PMD_SHIFT	20
# define PUD_SHIFT	31
# define PGDIR_SHIFT	42
#endif /* __s390x__ */

#define PMD_SIZE        (1UL << PMD_SHIFT)
#define PMD_MASK        (~(PMD_SIZE-1))
#define PUD_SIZE	(1UL << PUD_SHIFT)
#define PUD_MASK	(~(PUD_SIZE-1))
#define PGDIR_SIZE	(1UL << PGDIR_SHIFT)
#define PGDIR_MASK	(~(PGDIR_SIZE-1))

#define PTRS_PER_PTE	256
#ifndef __s390x__
#define PTRS_PER_PMD	1
#define PTRS_PER_PUD	1
#else /* __s390x__ */
#define PTRS_PER_PMD	2048
#define PTRS_PER_PUD	2048
#endif /* __s390x__ */
#define PTRS_PER_PGD	2048

#define FIRST_USER_ADDRESS  0

#define pte_ERROR(e) \
	printk("%s:%d: bad pte %p.\n", __FILE__, __LINE__, (void *) pte_val(e))
#define pmd_ERROR(e) \
	printk("%s:%d: bad pmd %p.\n", __FILE__, __LINE__, (void *) pmd_val(e))
#define pud_ERROR(e) \
	printk("%s:%d: bad pud %p.\n", __FILE__, __LINE__, (void *) pud_val(e))
#define pgd_ERROR(e) \
	printk("%s:%d: bad pgd %p.\n", __FILE__, __LINE__, (void *) pgd_val(e))

#ifndef __ASSEMBLY__

extern unsigned long VMALLOC_START;

#ifndef __s390x__
#define VMALLOC_SIZE	(96UL << 20)
#define VMALLOC_END	0x7e000000UL
#define VMEM_MAP_END	0x80000000UL
#else /* __s390x__ */
#define VMALLOC_SIZE	(128UL << 30)
#define VMALLOC_END	0x3e000000000UL
#define VMEM_MAP_END	0x40000000000UL
#endif /* __s390x__ */

#define VMEM_MAX_PAGES	((VMEM_MAP_END - VMALLOC_END) / sizeof(struct page))
#define VMEM_MAX_PFN	min(VMALLOC_START >> PAGE_SHIFT, VMEM_MAX_PAGES)
#define VMEM_MAX_PHYS	((VMEM_MAX_PFN << PAGE_SHIFT) & ~((16 << 20) - 1))
#define vmemmap		((struct page *) VMALLOC_END)


/* Hardware bits in the page table entry */
#define _PAGE_CO	0x100		/* HW Change-bit override */
#define _PAGE_RO	0x200		/* HW read-only bit  */
#define _PAGE_INVALID	0x400		/* HW invalid bit    */

/* Software bits in the page table entry */
#define _PAGE_SWT	0x001		/* SW pte type bit t */
#define _PAGE_SWX	0x002		/* SW pte type bit x */
#define _PAGE_SPECIAL	0x004		/* SW associated with special page */
#define __HAVE_ARCH_PTE_SPECIAL

/* Set of bits not changed in pte_modify */
#define _PAGE_CHG_MASK	(PAGE_MASK | _PAGE_SPECIAL)

/* Six different types of pages. */
#define _PAGE_TYPE_EMPTY	0x400
#define _PAGE_TYPE_NONE		0x401
#define _PAGE_TYPE_SWAP		0x403
#define _PAGE_TYPE_FILE		0x601	/* bit 0x002 is used for offset !! */
#define _PAGE_TYPE_RO		0x200
#define _PAGE_TYPE_RW		0x000
#define _PAGE_TYPE_EX_RO	0x202
#define _PAGE_TYPE_EX_RW	0x002

#define _HPAGE_TYPE_EMPTY	0x020	/* _SEGMENT_ENTRY_INV */
#define _HPAGE_TYPE_NONE	0x220
#define _HPAGE_TYPE_RO		0x200	/* _SEGMENT_ENTRY_RO  */
#define _HPAGE_TYPE_RW		0x000


/* Page status table bits for virtualization */
#define RCP_PCL_BIT	55
#define RCP_HR_BIT	54
#define RCP_HC_BIT	53
#define RCP_GR_BIT	50
#define RCP_GC_BIT	49

/* User dirty bit for KVM's migration feature */
#define KVM_UD_BIT	47

#ifndef __s390x__

/* Bits in the segment table address-space-control-element */
#define _ASCE_SPACE_SWITCH	0x80000000UL	/* space switch event	    */
#define _ASCE_ORIGIN_MASK	0x7ffff000UL	/* segment table origin	    */
#define _ASCE_PRIVATE_SPACE	0x100	/* private space control	    */
#define _ASCE_ALT_EVENT		0x80	/* storage alteration event control */
#define _ASCE_TABLE_LENGTH	0x7f	/* 128 x 64 entries = 8k	    */

/* Bits in the segment table entry */
#define _SEGMENT_ENTRY_ORIGIN	0x7fffffc0UL	/* page table origin	    */
#define _SEGMENT_ENTRY_INV	0x20	/* invalid segment table entry	    */
#define _SEGMENT_ENTRY_COMMON	0x10	/* common segment bit		    */
#define _SEGMENT_ENTRY_PTL	0x0f	/* page table length		    */

#define _SEGMENT_ENTRY		(_SEGMENT_ENTRY_PTL)
#define _SEGMENT_ENTRY_EMPTY	(_SEGMENT_ENTRY_INV)

#else /* __s390x__ */

/* Bits in the segment/region table address-space-control-element */
#define _ASCE_ORIGIN		~0xfffUL/* segment table origin		    */
#define _ASCE_PRIVATE_SPACE	0x100	/* private space control	    */
#define _ASCE_ALT_EVENT		0x80	/* storage alteration event control */
#define _ASCE_SPACE_SWITCH	0x40	/* space switch event		    */
#define _ASCE_REAL_SPACE	0x20	/* real space control		    */
#define _ASCE_TYPE_MASK		0x0c	/* asce table type mask		    */
#define _ASCE_TYPE_REGION1	0x0c	/* region first table type	    */
#define _ASCE_TYPE_REGION2	0x08	/* region second table type	    */
#define _ASCE_TYPE_REGION3	0x04	/* region third table type	    */
#define _ASCE_TYPE_SEGMENT	0x00	/* segment table type		    */
#define _ASCE_TABLE_LENGTH	0x03	/* region table length		    */

/* Bits in the region table entry */
#define _REGION_ENTRY_ORIGIN	~0xfffUL/* region/segment table origin	    */
#define _REGION_ENTRY_INV	0x20	/* invalid region table entry	    */
#define _REGION_ENTRY_TYPE_MASK	0x0c	/* region/segment table type mask   */
#define _REGION_ENTRY_TYPE_R1	0x0c	/* region first table type	    */
#define _REGION_ENTRY_TYPE_R2	0x08	/* region second table type	    */
#define _REGION_ENTRY_TYPE_R3	0x04	/* region third table type	    */
#define _REGION_ENTRY_LENGTH	0x03	/* region third length		    */

#define _REGION1_ENTRY		(_REGION_ENTRY_TYPE_R1 | _REGION_ENTRY_LENGTH)
#define _REGION1_ENTRY_EMPTY	(_REGION_ENTRY_TYPE_R1 | _REGION_ENTRY_INV)
#define _REGION2_ENTRY		(_REGION_ENTRY_TYPE_R2 | _REGION_ENTRY_LENGTH)
#define _REGION2_ENTRY_EMPTY	(_REGION_ENTRY_TYPE_R2 | _REGION_ENTRY_INV)
#define _REGION3_ENTRY		(_REGION_ENTRY_TYPE_R3 | _REGION_ENTRY_LENGTH)
#define _REGION3_ENTRY_EMPTY	(_REGION_ENTRY_TYPE_R3 | _REGION_ENTRY_INV)

/* Bits in the segment table entry */
#define _SEGMENT_ENTRY_ORIGIN	~0x7ffUL/* segment table origin		    */
#define _SEGMENT_ENTRY_RO	0x200	/* page protection bit		    */
#define _SEGMENT_ENTRY_INV	0x20	/* invalid segment table entry	    */

#define _SEGMENT_ENTRY		(0)
#define _SEGMENT_ENTRY_EMPTY	(_SEGMENT_ENTRY_INV)

#define _SEGMENT_ENTRY_LARGE	0x400	/* STE-format control, large page   */
#define _SEGMENT_ENTRY_CO	0x100	/* change-recording override   */

#endif /* __s390x__ */

#define _ASCE_USER_BITS		(_ASCE_SPACE_SWITCH | _ASCE_PRIVATE_SPACE | \
				 _ASCE_ALT_EVENT)

/* Bits int the storage key */
#define _PAGE_CHANGED    0x02          /* HW changed bit                   */
#define _PAGE_REFERENCED 0x04          /* HW referenced bit                */

#define PAGE_NONE	__pgprot(_PAGE_TYPE_NONE)
#define PAGE_RO		__pgprot(_PAGE_TYPE_RO)
#define PAGE_RW		__pgprot(_PAGE_TYPE_RW)
#define PAGE_EX_RO	__pgprot(_PAGE_TYPE_EX_RO)
#define PAGE_EX_RW	__pgprot(_PAGE_TYPE_EX_RW)

#define PAGE_KERNEL	PAGE_RW
#define PAGE_COPY	PAGE_RO

         /*xwr*/
#define __P000	PAGE_NONE
#define __P001	PAGE_RO
#define __P010	PAGE_RO
#define __P011	PAGE_RO
#define __P100	PAGE_EX_RO
#define __P101	PAGE_EX_RO
#define __P110	PAGE_EX_RO
#define __P111	PAGE_EX_RO

#define __S000	PAGE_NONE
#define __S001	PAGE_RO
#define __S010	PAGE_RW
#define __S011	PAGE_RW
#define __S100	PAGE_EX_RO
#define __S101	PAGE_EX_RO
#define __S110	PAGE_EX_RW
#define __S111	PAGE_EX_RW

#ifndef __s390x__
# define PxD_SHADOW_SHIFT	1
#else /* __s390x__ */
# define PxD_SHADOW_SHIFT	2
#endif /* __s390x__ */

static inline void *get_shadow_table(void *table)
{
	unsigned long addr, offset;
	struct page *page;

	addr = (unsigned long) table;
	offset = addr & ((PAGE_SIZE << PxD_SHADOW_SHIFT) - 1);
	page = virt_to_page((void *)(addr ^ offset));
	return (void *)(addr_t)(page->index ? (page->index | offset) : 0UL);
}

static inline void set_pte_at(struct mm_struct *mm, unsigned long addr,
			      pte_t *ptep, pte_t entry)
{
	*ptep = entry;
	if (mm->context.noexec) {
		if (!(pte_val(entry) & _PAGE_INVALID) &&
		    (pte_val(entry) & _PAGE_SWX))
			pte_val(entry) |= _PAGE_RO;
		else
			pte_val(entry) = _PAGE_TYPE_EMPTY;
		ptep[PTRS_PER_PTE] = entry;
	}
}

#ifndef __s390x__

static inline int pgd_present(pgd_t pgd) { return 1; }
static inline int pgd_none(pgd_t pgd)    { return 0; }
static inline int pgd_bad(pgd_t pgd)     { return 0; }

static inline int pud_present(pud_t pud) { return 1; }
static inline int pud_none(pud_t pud)	 { return 0; }
static inline int pud_bad(pud_t pud)	 { return 0; }

#else /* __s390x__ */

static inline int pgd_present(pgd_t pgd)
{
	if ((pgd_val(pgd) & _REGION_ENTRY_TYPE_MASK) < _REGION_ENTRY_TYPE_R2)
		return 1;
	return (pgd_val(pgd) & _REGION_ENTRY_ORIGIN) != 0UL;
}

static inline int pgd_none(pgd_t pgd)
{
	if ((pgd_val(pgd) & _REGION_ENTRY_TYPE_MASK) < _REGION_ENTRY_TYPE_R2)
		return 0;
	return (pgd_val(pgd) & _REGION_ENTRY_INV) != 0UL;
}

static inline int pgd_bad(pgd_t pgd)
{
	/*
	 * With dynamic page table levels the pgd can be a region table
	 * entry or a segment table entry. Check for the bit that are
	 * invalid for either table entry.
	 */
	unsigned long mask =
		~_SEGMENT_ENTRY_ORIGIN & ~_REGION_ENTRY_INV &
		~_REGION_ENTRY_TYPE_MASK & ~_REGION_ENTRY_LENGTH;
	return (pgd_val(pgd) & mask) != 0;
}

static inline int pud_present(pud_t pud)
{
	if ((pud_val(pud) & _REGION_ENTRY_TYPE_MASK) < _REGION_ENTRY_TYPE_R3)
		return 1;
	return (pud_val(pud) & _REGION_ENTRY_ORIGIN) != 0UL;
}

static inline int pud_none(pud_t pud)
{
	if ((pud_val(pud) & _REGION_ENTRY_TYPE_MASK) < _REGION_ENTRY_TYPE_R3)
		return 0;
	return (pud_val(pud) & _REGION_ENTRY_INV) != 0UL;
}

static inline int pud_bad(pud_t pud)
{
	/*
	 * With dynamic page table levels the pud can be a region table
	 * entry or a segment table entry. Check for the bit that are
	 * invalid for either table entry.
	 */
	unsigned long mask =
		~_SEGMENT_ENTRY_ORIGIN & ~_REGION_ENTRY_INV &
		~_REGION_ENTRY_TYPE_MASK & ~_REGION_ENTRY_LENGTH;
	return (pud_val(pud) & mask) != 0;
}

#endif /* __s390x__ */

static inline int pmd_present(pmd_t pmd)
{
	return (pmd_val(pmd) & _SEGMENT_ENTRY_ORIGIN) != 0UL;
}

static inline int pmd_none(pmd_t pmd)
{
	return (pmd_val(pmd) & _SEGMENT_ENTRY_INV) != 0UL;
}

static inline int pmd_bad(pmd_t pmd)
{
	unsigned long mask = ~_SEGMENT_ENTRY_ORIGIN & ~_SEGMENT_ENTRY_INV;
	return (pmd_val(pmd) & mask) != _SEGMENT_ENTRY;
}

static inline int pte_none(pte_t pte)
{
	return (pte_val(pte) & _PAGE_INVALID) && !(pte_val(pte) & _PAGE_SWT);
}

static inline int pte_present(pte_t pte)
{
	unsigned long mask = _PAGE_RO | _PAGE_INVALID | _PAGE_SWT | _PAGE_SWX;
	return (pte_val(pte) & mask) == _PAGE_TYPE_NONE ||
		(!(pte_val(pte) & _PAGE_INVALID) &&
		 !(pte_val(pte) & _PAGE_SWT));
}

static inline int pte_file(pte_t pte)
{
	unsigned long mask = _PAGE_RO | _PAGE_INVALID | _PAGE_SWT;
	return (pte_val(pte) & mask) == _PAGE_TYPE_FILE;
}

static inline int pte_special(pte_t pte)
{
	return (pte_val(pte) & _PAGE_SPECIAL);
}

#define __HAVE_ARCH_PTE_SAME
#define pte_same(a,b)  (pte_val(a) == pte_val(b))

static inline void rcp_lock(pte_t *ptep)
{
#ifdef CONFIG_PGSTE
	unsigned long *pgste = (unsigned long *) (ptep + PTRS_PER_PTE);
	preempt_disable();
	while (test_and_set_bit(RCP_PCL_BIT, pgste))
		;
#endif
}

static inline void rcp_unlock(pte_t *ptep)
{
#ifdef CONFIG_PGSTE
	unsigned long *pgste = (unsigned long *) (ptep + PTRS_PER_PTE);
	clear_bit(RCP_PCL_BIT, pgste);
	preempt_enable();
#endif
}

/* forward declaration for SetPageUptodate in page-flags.h*/
static inline void page_clear_dirty(struct page *page);
#include <linux/page-flags.h>

static inline void ptep_rcp_copy(pte_t *ptep)
{
#ifdef CONFIG_PGSTE
	struct page *page = virt_to_page(pte_val(*ptep));
	unsigned int skey;
	unsigned long *pgste = (unsigned long *) (ptep + PTRS_PER_PTE);

	skey = page_get_storage_key(page_to_phys(page));
	if (skey & _PAGE_CHANGED) {
		set_bit_simple(RCP_GC_BIT, pgste);
		set_bit_simple(KVM_UD_BIT, pgste);
	}
	if (skey & _PAGE_REFERENCED)
		set_bit_simple(RCP_GR_BIT, pgste);
	if (test_and_clear_bit_simple(RCP_HC_BIT, pgste)) {
		SetPageDirty(page);
		set_bit_simple(KVM_UD_BIT, pgste);
	}
	if (test_and_clear_bit_simple(RCP_HR_BIT, pgste))
		SetPageReferenced(page);
#endif
}

static inline int pte_write(pte_t pte)
{
	return (pte_val(pte) & _PAGE_RO) == 0;
}

static inline int pte_dirty(pte_t pte)
{
	/* A pte is neither clean nor dirty on s/390. The dirty bit
	 * is in the storage key. See page_test_and_clear_dirty for
	 * details.
	 */
	return 0;
}

static inline int pte_young(pte_t pte)
{
	/* A pte is neither young nor old on s/390. The young bit
	 * is in the storage key. See page_test_and_clear_young for
	 * details.
	 */
	return 0;
}


#ifndef __s390x__

#define pgd_clear(pgd)		do { } while (0)
#define pud_clear(pud)		do { } while (0)

#else /* __s390x__ */

static inline void pgd_clear_kernel(pgd_t * pgd)
{
	if ((pgd_val(*pgd) & _REGION_ENTRY_TYPE_MASK) == _REGION_ENTRY_TYPE_R2)
		pgd_val(*pgd) = _REGION2_ENTRY_EMPTY;
}

static inline void pgd_clear(pgd_t * pgd)
{
	pgd_t *shadow = get_shadow_table(pgd);

	pgd_clear_kernel(pgd);
	if (shadow)
		pgd_clear_kernel(shadow);
}

static inline void pud_clear_kernel(pud_t *pud)
{
	if ((pud_val(*pud) & _REGION_ENTRY_TYPE_MASK) == _REGION_ENTRY_TYPE_R3)
		pud_val(*pud) = _REGION3_ENTRY_EMPTY;
}

static inline void pud_clear(pud_t *pud)
{
	pud_t *shadow = get_shadow_table(pud);

	pud_clear_kernel(pud);
	if (shadow)
		pud_clear_kernel(shadow);
}

#endif /* __s390x__ */

static inline void pmd_clear_kernel(pmd_t * pmdp)
{
	pmd_val(*pmdp) = _SEGMENT_ENTRY_EMPTY;
}

static inline void pmd_clear(pmd_t *pmd)
{
	pmd_t *shadow = get_shadow_table(pmd);

	pmd_clear_kernel(pmd);
	if (shadow)
		pmd_clear_kernel(shadow);
}

static inline void pte_clear(struct mm_struct *mm, unsigned long addr, pte_t *ptep)
{
	pte_val(*ptep) = _PAGE_TYPE_EMPTY;
	if (mm->context.noexec)
		pte_val(ptep[PTRS_PER_PTE]) = _PAGE_TYPE_EMPTY;
}

static inline pte_t pte_modify(pte_t pte, pgprot_t newprot)
{
	pte_val(pte) &= _PAGE_CHG_MASK;
	pte_val(pte) |= pgprot_val(newprot);
	return pte;
}

static inline pte_t pte_wrprotect(pte_t pte)
{
	/* Do not clobber _PAGE_TYPE_NONE pages!  */
	if (!(pte_val(pte) & _PAGE_INVALID))
		pte_val(pte) |= _PAGE_RO;
	return pte;
}

static inline pte_t pte_mkwrite(pte_t pte)
{
	pte_val(pte) &= ~_PAGE_RO;
	return pte;
}

static inline pte_t pte_mkclean(pte_t pte)
{
	/* The only user of pte_mkclean is the fork() code.
	   We must *not* clear the *physical* page dirty bit
	   just because fork() wants to clear the dirty bit in
	   *one* of the page's mappings.  So we just do nothing. */
	return pte;
}

static inline pte_t pte_mkdirty(pte_t pte)
{
	/* We do not explicitly set the dirty bit because the
	 * sske instruction is slow. It is faster to let the
	 * next instruction set the dirty bit.
	 */
	return pte;
}

static inline pte_t pte_mkold(pte_t pte)
{
	/* S/390 doesn't keep its dirty/referenced bit in the pte.
	 * There is no point in clearing the real referenced bit.
	 */
	return pte;
}

static inline pte_t pte_mkyoung(pte_t pte)
{
	/* S/390 doesn't keep its dirty/referenced bit in the pte.
	 * There is no point in setting the real referenced bit.
	 */
	return pte;
}

static inline pte_t pte_mkspecial(pte_t pte)
{
	pte_val(pte) |= _PAGE_SPECIAL;
	return pte;
}

#ifdef CONFIG_PGSTE
static inline int kvm_s390_test_and_clear_page_dirty(struct mm_struct *mm,
						     pte_t *ptep)
{
	int dirty;
	unsigned long *pgste;
	struct page *page;
	unsigned int skey;

	if (!mm->context.has_pgste)
		return -EINVAL;
	rcp_lock(ptep);
	pgste = (unsigned long *) (ptep + PTRS_PER_PTE);
	page = virt_to_page(pte_val(*ptep));
	skey = page_get_storage_key(page_to_phys(page));
	if (skey & _PAGE_CHANGED) {
		set_bit_simple(RCP_GC_BIT, pgste);
		set_bit_simple(KVM_UD_BIT, pgste);
	}
	if (test_and_clear_bit_simple(RCP_HC_BIT, pgste)) {
		SetPageDirty(page);
		set_bit_simple(KVM_UD_BIT, pgste);
	}
	dirty = test_and_clear_bit_simple(KVM_UD_BIT, pgste);
	if (skey & _PAGE_CHANGED)
		page_clear_dirty(page);
	rcp_unlock(ptep);
	return dirty;
}
#endif

#define __HAVE_ARCH_PTEP_TEST_AND_CLEAR_YOUNG
static inline int ptep_test_and_clear_young(struct vm_area_struct *vma,
					    unsigned long addr, pte_t *ptep)
{
#ifdef CONFIG_PGSTE
	unsigned long physpage;
	int young;
	unsigned long *pgste;

	if (!vma->vm_mm->context.has_pgste)
		return 0;
	physpage = pte_val(*ptep) & PAGE_MASK;
	pgste = (unsigned long *) (ptep + PTRS_PER_PTE);

	young = ((page_get_storage_key(physpage) & _PAGE_REFERENCED) != 0);
	rcp_lock(ptep);
	if (young)
		set_bit_simple(RCP_GR_BIT, pgste);
	young |= test_and_clear_bit_simple(RCP_HR_BIT, pgste);
	rcp_unlock(ptep);
	return young;
#endif
	return 0;
}

#define __HAVE_ARCH_PTEP_CLEAR_YOUNG_FLUSH
static inline int ptep_clear_flush_young(struct vm_area_struct *vma,
					 unsigned long address, pte_t *ptep)
{
	/* No need to flush TLB
	 * On s390 reference bits are in storage key and never in TLB
	 * With virtualization we handle the reference bit, without we
	 * we can simply return */
#ifdef CONFIG_PGSTE
	return ptep_test_and_clear_young(vma, address, ptep);
#endif
	return 0;
}

static inline void __ptep_ipte(unsigned long address, pte_t *ptep)
{
	if (!(pte_val(*ptep) & _PAGE_INVALID)) {
#ifndef __s390x__
		/* pto must point to the start of the segment table */
		pte_t *pto = (pte_t *) (((unsigned long) ptep) & 0x7ffffc00);
#else
		/* ipte in zarch mode can do the math */
		pte_t *pto = ptep;
#endif
		asm volatile(
			"	ipte	%2,%3"
			: "=m" (*ptep) : "m" (*ptep),
			  "a" (pto), "a" (address));
	}
}

static inline void ptep_invalidate(struct mm_struct *mm,
				   unsigned long address, pte_t *ptep)
{
	if (mm->context.has_pgste) {
		rcp_lock(ptep);
		__ptep_ipte(address, ptep);
		ptep_rcp_copy(ptep);
		pte_val(*ptep) = _PAGE_TYPE_EMPTY;
		rcp_unlock(ptep);
		return;
	}
	__ptep_ipte(address, ptep);
	pte_val(*ptep) = _PAGE_TYPE_EMPTY;
	if (mm->context.noexec) {
		__ptep_ipte(address, ptep + PTRS_PER_PTE);
		pte_val(*(ptep + PTRS_PER_PTE)) = _PAGE_TYPE_EMPTY;
	}
}

#define __HAVE_ARCH_PTEP_GET_AND_CLEAR
#define ptep_get_and_clear(__mm, __address, __ptep)			\
({									\
	pte_t __pte = *(__ptep);					\
	if (atomic_read(&(__mm)->mm_users) > 1 ||			\
	    (__mm) != current->active_mm)				\
		ptep_invalidate(__mm, __address, __ptep);		\
	else								\
		pte_clear((__mm), (__address), (__ptep));		\
	__pte;								\
})

#define __HAVE_ARCH_PTEP_CLEAR_FLUSH
static inline pte_t ptep_clear_flush(struct vm_area_struct *vma,
				     unsigned long address, pte_t *ptep)
{
	pte_t pte = *ptep;
	ptep_invalidate(vma->vm_mm, address, ptep);
	return pte;
}

#define __HAVE_ARCH_PTEP_GET_AND_CLEAR_FULL
static inline pte_t ptep_get_and_clear_full(struct mm_struct *mm,
					    unsigned long addr,
					    pte_t *ptep, int full)
{
	pte_t pte = *ptep;

	if (full)
		pte_clear(mm, addr, ptep);
	else
		ptep_invalidate(mm, addr, ptep);
	return pte;
}

#define __HAVE_ARCH_PTEP_SET_WRPROTECT
#define ptep_set_wrprotect(__mm, __addr, __ptep)			\
({									\
	pte_t __pte = *(__ptep);					\
	if (pte_write(__pte)) {						\
		if (atomic_read(&(__mm)->mm_users) > 1 ||		\
		    (__mm) != current->active_mm)			\
			ptep_invalidate(__mm, __addr, __ptep);		\
		set_pte_at(__mm, __addr, __ptep, pte_wrprotect(__pte));	\
	}								\
})

#define __HAVE_ARCH_PTEP_SET_ACCESS_FLAGS
#define ptep_set_access_flags(__vma, __addr, __ptep, __entry, __dirty)	\
({									\
	int __changed = !pte_same(*(__ptep), __entry);			\
	if (__changed) {						\
		ptep_invalidate((__vma)->vm_mm, __addr, __ptep);	\
		set_pte_at((__vma)->vm_mm, __addr, __ptep, __entry);	\
	}								\
	__changed;							\
})

#define __HAVE_ARCH_PAGE_TEST_DIRTY
static inline int page_test_dirty(struct page *page)
{
	return (page_get_storage_key(page_to_phys(page)) & _PAGE_CHANGED) != 0;
}

#define __HAVE_ARCH_PAGE_CLEAR_DIRTY
static inline void page_clear_dirty(struct page *page)
{
	page_set_storage_key(page_to_phys(page), PAGE_DEFAULT_KEY);
}

#define __HAVE_ARCH_PAGE_TEST_AND_CLEAR_YOUNG
static inline int page_test_and_clear_young(struct page *page)
{
	unsigned long physpage = page_to_phys(page);
	int ccode;

	asm volatile(
		"	rrbe	0,%1\n"
		"	ipm	%0\n"
		"	srl	%0,28\n"
		: "=d" (ccode) : "a" (physpage) : "cc" );
	return ccode & 2;
}

static inline pte_t mk_pte_phys(unsigned long physpage, pgprot_t pgprot)
{
	pte_t __pte;
	pte_val(__pte) = physpage + pgprot_val(pgprot);
	return __pte;
}

static inline pte_t mk_pte(struct page *page, pgprot_t pgprot)
{
	unsigned long physpage = page_to_phys(page);

	return mk_pte_phys(physpage, pgprot);
}

#define pgd_index(address) (((address) >> PGDIR_SHIFT) & (PTRS_PER_PGD-1))
#define pud_index(address) (((address) >> PUD_SHIFT) & (PTRS_PER_PUD-1))
#define pmd_index(address) (((address) >> PMD_SHIFT) & (PTRS_PER_PMD-1))
#define pte_index(address) (((address) >> PAGE_SHIFT) & (PTRS_PER_PTE-1))

#define pgd_offset(mm, address) ((mm)->pgd + pgd_index(address))
#define pgd_offset_k(address) pgd_offset(&init_mm, address)

#ifndef __s390x__

#define pmd_deref(pmd) (pmd_val(pmd) & _SEGMENT_ENTRY_ORIGIN)
#define pud_deref(pmd) ({ BUG(); 0UL; })
#define pgd_deref(pmd) ({ BUG(); 0UL; })

#define pud_offset(pgd, address) ((pud_t *) pgd)
#define pmd_offset(pud, address) ((pmd_t *) pud + pmd_index(address))

#else /* __s390x__ */

#define pmd_deref(pmd) (pmd_val(pmd) & _SEGMENT_ENTRY_ORIGIN)
#define pud_deref(pud) (pud_val(pud) & _REGION_ENTRY_ORIGIN)
#define pgd_deref(pgd) (pgd_val(pgd) & _REGION_ENTRY_ORIGIN)

static inline pud_t *pud_offset(pgd_t *pgd, unsigned long address)
{
	pud_t *pud = (pud_t *) pgd;
	if ((pgd_val(*pgd) & _REGION_ENTRY_TYPE_MASK) == _REGION_ENTRY_TYPE_R2)
		pud = (pud_t *) pgd_deref(*pgd);
	return pud  + pud_index(address);
}

static inline pmd_t *pmd_offset(pud_t *pud, unsigned long address)
{
	pmd_t *pmd = (pmd_t *) pud;
	if ((pud_val(*pud) & _REGION_ENTRY_TYPE_MASK) == _REGION_ENTRY_TYPE_R3)
		pmd = (pmd_t *) pud_deref(*pud);
	return pmd + pmd_index(address);
}

#endif /* __s390x__ */

#define pfn_pte(pfn,pgprot) mk_pte_phys(__pa((pfn) << PAGE_SHIFT),(pgprot))
#define pte_pfn(x) (pte_val(x) >> PAGE_SHIFT)
#define pte_page(x) pfn_to_page(pte_pfn(x))

#define pmd_page(pmd) pfn_to_page(pmd_val(pmd) >> PAGE_SHIFT)

/* Find an entry in the lowest level page table.. */
#define pte_offset(pmd, addr) ((pte_t *) pmd_deref(*(pmd)) + pte_index(addr))
#define pte_offset_kernel(pmd, address) pte_offset(pmd,address)
#define pte_offset_map(pmd, address) pte_offset_kernel(pmd, address)
#define pte_offset_map_nested(pmd, address) pte_offset_kernel(pmd, address)
#define pte_unmap(pte) do { } while (0)
#define pte_unmap_nested(pte) do { } while (0)

#ifndef __s390x__
#define __SWP_OFFSET_MASK (~0UL >> 12)
#else
#define __SWP_OFFSET_MASK (~0UL >> 11)
#endif
static inline pte_t mk_swap_pte(unsigned long type, unsigned long offset)
{
	pte_t pte;
	offset &= __SWP_OFFSET_MASK;
	pte_val(pte) = _PAGE_TYPE_SWAP | ((type & 0x1f) << 2) |
		((offset & 1UL) << 7) | ((offset & ~1UL) << 11);
	return pte;
}

#define __swp_type(entry)	(((entry).val >> 2) & 0x1f)
#define __swp_offset(entry)	(((entry).val >> 11) | (((entry).val >> 7) & 1))
#define __swp_entry(type,offset) ((swp_entry_t) { pte_val(mk_swap_pte((type),(offset))) })

#define __pte_to_swp_entry(pte)	((swp_entry_t) { pte_val(pte) })
#define __swp_entry_to_pte(x)	((pte_t) { (x).val })

#ifndef __s390x__
# define PTE_FILE_MAX_BITS	26
#else /* __s390x__ */
# define PTE_FILE_MAX_BITS	59
#endif /* __s390x__ */

#define pte_to_pgoff(__pte) \
	((((__pte).pte >> 12) << 7) + (((__pte).pte >> 1) & 0x7f))

#define pgoff_to_pte(__off) \
	((pte_t) { ((((__off) & 0x7f) << 1) + (((__off) >> 7) << 12)) \
		   | _PAGE_TYPE_FILE })

#endif /* !__ASSEMBLY__ */

#define kern_addr_valid(addr)   (1)

extern int vmem_add_mapping(unsigned long start, unsigned long size);
extern int vmem_remove_mapping(unsigned long start, unsigned long size);
extern int s390_enable_sie(void);

#define pgtable_cache_init()	do { } while (0)

#include <asm-generic/pgtable.h>

#endif /* _S390_PAGE_H */
