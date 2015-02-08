#ifndef _ASMARM_PGTABLE_H
#define _ASMARM_PGTABLE_H

#include <asm-generic/4level-fixup.h>
#include <asm/proc-fns.h>

#ifndef CONFIG_MMU

#include "pgtable-nommu.h"

#else

#include <asm/memory.h>
#include <asm/arch/vmalloc.h>

#ifndef VMALLOC_START
#define VMALLOC_OFFSET		(8*1024*1024)
#define VMALLOC_START		(((unsigned long)high_memory + VMALLOC_OFFSET) & ~(VMALLOC_OFFSET-1))
#endif

#define PTRS_PER_PTE		512
#define PTRS_PER_PMD		1
#define PTRS_PER_PGD		2048

#define PMD_SHIFT		21
#define PGDIR_SHIFT		21

#define LIBRARY_TEXT_START	0x0c000000

#ifndef __ASSEMBLY__
extern void __pte_error(const char *file, int line, unsigned long val);
extern void __pmd_error(const char *file, int line, unsigned long val);
extern void __pgd_error(const char *file, int line, unsigned long val);

#define pte_ERROR(pte)		__pte_error(__FILE__, __LINE__, pte_val(pte))
#define pmd_ERROR(pmd)		__pmd_error(__FILE__, __LINE__, pmd_val(pmd))
#define pgd_ERROR(pgd)		__pgd_error(__FILE__, __LINE__, pgd_val(pgd))
#endif /* !__ASSEMBLY__ */

#define PMD_SIZE		(1UL << PMD_SHIFT)
#define PMD_MASK		(~(PMD_SIZE-1))
#define PGDIR_SIZE		(1UL << PGDIR_SHIFT)
#define PGDIR_MASK		(~(PGDIR_SIZE-1))

#define FIRST_USER_ADDRESS	PAGE_SIZE

#define FIRST_USER_PGD_NR	1
#define USER_PTRS_PER_PGD	((TASK_SIZE/PGDIR_SIZE) - FIRST_USER_PGD_NR)

#define SUPERSECTION_SHIFT	24
#define SUPERSECTION_SIZE	(1UL << SUPERSECTION_SHIFT)
#define SUPERSECTION_MASK	(~(SUPERSECTION_SIZE-1))

#define L_PTE_PRESENT		(1 << 0)
#define L_PTE_FILE		(1 << 1)	/* only when !PRESENT */
#define L_PTE_YOUNG		(1 << 1)
#define L_PTE_BUFFERABLE	(1 << 2)	/* matches PTE */
#define L_PTE_CACHEABLE		(1 << 3)	/* matches PTE */
#define L_PTE_USER		(1 << 4)
#define L_PTE_WRITE		(1 << 5)
#define L_PTE_EXEC		(1 << 6)
#define L_PTE_DIRTY		(1 << 7)
#define L_PTE_COHERENT		(1 << 9)	/* I/O coherent (xsc3) */
#define L_PTE_SHARED		(1 << 10)	/* shared between CPUs (v6) */
#define L_PTE_ASID		(1 << 11)	/* non-global (use ASID, v6) */

#ifndef __ASSEMBLY__

#define _L_PTE_DEFAULT	L_PTE_PRESENT | L_PTE_YOUNG | L_PTE_CACHEABLE | L_PTE_BUFFERABLE
#define _L_PTE_READ	L_PTE_USER | L_PTE_EXEC

extern pgprot_t		pgprot_kernel;

#define PAGE_NONE       __pgprot(_L_PTE_DEFAULT)
#define PAGE_COPY       __pgprot(_L_PTE_DEFAULT | _L_PTE_READ)
#define PAGE_SHARED     __pgprot(_L_PTE_DEFAULT | _L_PTE_READ | L_PTE_WRITE)
#define PAGE_READONLY   __pgprot(_L_PTE_DEFAULT | _L_PTE_READ)
#define PAGE_KERNEL	pgprot_kernel

#endif /* __ASSEMBLY__ */

#define __P000  PAGE_NONE
#define __P001  PAGE_READONLY
#define __P010  PAGE_COPY
#define __P011  PAGE_COPY
#define __P100  PAGE_READONLY
#define __P101  PAGE_READONLY
#define __P110  PAGE_COPY
#define __P111  PAGE_COPY

#define __S000  PAGE_NONE
#define __S001  PAGE_READONLY
#define __S010  PAGE_SHARED
#define __S011  PAGE_SHARED
#define __S100  PAGE_READONLY
#define __S101  PAGE_READONLY
#define __S110  PAGE_SHARED
#define __S111  PAGE_SHARED

#ifndef __ASSEMBLY__
extern struct page *empty_zero_page;
#define ZERO_PAGE(vaddr)	(empty_zero_page)

#define pte_pfn(pte)		(pte_val(pte) >> PAGE_SHIFT)
#define pfn_pte(pfn,prot)	(__pte(((pfn) << PAGE_SHIFT) | pgprot_val(prot)))

#define pte_none(pte)		(!pte_val(pte))
#define pte_clear(mm,addr,ptep)	set_pte_at((mm),(addr),(ptep), __pte(0))
#define pte_page(pte)		(pfn_to_page(pte_pfn(pte)))
#define pte_offset_kernel(dir,addr)	(pmd_page_kernel(*(dir)) + __pte_index(addr))
#define pte_offset_map(dir,addr)	(pmd_page_kernel(*(dir)) + __pte_index(addr))
#define pte_offset_map_nested(dir,addr)	(pmd_page_kernel(*(dir)) + __pte_index(addr))
#define pte_unmap(pte)		do { } while (0)
#define pte_unmap_nested(pte)	do { } while (0)

#define set_pte(ptep, pte)	cpu_set_pte(ptep,pte)
#define set_pte_at(mm,addr,ptep,pteval) set_pte(ptep,pteval)

#define pte_present(pte)	(pte_val(pte) & L_PTE_PRESENT)
#define pte_read(pte)		(pte_val(pte) & L_PTE_USER)
#define pte_write(pte)		(pte_val(pte) & L_PTE_WRITE)
#define pte_exec(pte)		(pte_val(pte) & L_PTE_EXEC)
#define pte_dirty(pte)		(pte_val(pte) & L_PTE_DIRTY)
#define pte_young(pte)		(pte_val(pte) & L_PTE_YOUNG)

#define pte_file(pte)		(pte_val(pte) & L_PTE_FILE)
#define pte_to_pgoff(x)		(pte_val(x) >> 2)
#define pgoff_to_pte(x)		__pte(((x) << 2) | L_PTE_FILE)

#define PTE_FILE_MAX_BITS	30

#define PTE_BIT_FUNC(fn,op) \
static inline pte_t pte_##fn(pte_t pte) { pte_val(pte) op; return pte; }

/*PTE_BIT_FUNC(rdprotect, &= ~L_PTE_USER);*/
/*PTE_BIT_FUNC(mkread,    |= L_PTE_USER);*/
PTE_BIT_FUNC(wrprotect, &= ~L_PTE_WRITE);
PTE_BIT_FUNC(mkwrite,   |= L_PTE_WRITE);
PTE_BIT_FUNC(exprotect, &= ~L_PTE_EXEC);
PTE_BIT_FUNC(mkexec,    |= L_PTE_EXEC);
PTE_BIT_FUNC(mkclean,   &= ~L_PTE_DIRTY);
PTE_BIT_FUNC(mkdirty,   |= L_PTE_DIRTY);
PTE_BIT_FUNC(mkold,     &= ~L_PTE_YOUNG);
PTE_BIT_FUNC(mkyoung,   |= L_PTE_YOUNG);

#define pgprot_noncached(prot)	__pgprot(pgprot_val(prot) & ~(L_PTE_CACHEABLE | L_PTE_BUFFERABLE))
#define pgprot_writecombine(prot) __pgprot(pgprot_val(prot) & ~L_PTE_CACHEABLE)

#define pmd_none(pmd)		(!pmd_val(pmd))
#define pmd_present(pmd)	(pmd_val(pmd))
#define pmd_bad(pmd)		(pmd_val(pmd) & 2)

#define copy_pmd(pmdpd,pmdps)		\
	do {				\
		pmdpd[0] = pmdps[0];	\
		pmdpd[1] = pmdps[1];	\
		flush_pmd_entry(pmdpd);	\
	} while (0)

#define pmd_clear(pmdp)			\
	do {				\
		pmdp[0] = __pmd(0);	\
		pmdp[1] = __pmd(0);	\
		clean_pmd_entry(pmdp);	\
	} while (0)

static inline pte_t *pmd_page_kernel(pmd_t pmd)
{
	unsigned long ptr;

	ptr = pmd_val(pmd) & ~(PTRS_PER_PTE * sizeof(void *) - 1);
	ptr += PTRS_PER_PTE * sizeof(void *);

	return __va(ptr);
}

#define pmd_page(pmd) virt_to_page(__va(pmd_val(pmd)))

#define pages_to_mb(x)		((x) >> (20 - PAGE_SHIFT))

#define mk_pte(page,prot)	pfn_pte(page_to_pfn(page),prot)

#define pgd_none(pgd)		(0)
#define pgd_bad(pgd)		(0)
#define pgd_present(pgd)	(1)
#define pgd_clear(pgdp)		do { } while (0)
#define set_pgd(pgd,pgdp)	do { } while (0)

/* to find an entry in a page-table-directory */
#define pgd_index(addr)		((addr) >> PGDIR_SHIFT)

#define pgd_offset(mm, addr)	((mm)->pgd+pgd_index(addr))

/* to find an entry in a kernel page-table-directory */
#define pgd_offset_k(addr)	pgd_offset(&init_mm, addr)

/* Find an entry in the second-level page table.. */
#define pmd_offset(dir, addr)	((pmd_t *)(dir))

/* Find an entry in the third-level page table.. */
#define __pte_index(addr)	(((addr) >> PAGE_SHIFT) & (PTRS_PER_PTE - 1))

static inline pte_t pte_modify(pte_t pte, pgprot_t newprot)
{
	const unsigned long mask = L_PTE_EXEC | L_PTE_WRITE | L_PTE_USER;
	pte_val(pte) = (pte_val(pte) & ~mask) | (pgprot_val(newprot) & mask);
	return pte;
}

extern pgd_t swapper_pg_dir[PTRS_PER_PGD];

#define __swp_type(x)		(((x).val >> 2) & 0x7f)
#define __swp_offset(x)		((x).val >> 9)
#define __swp_entry(type,offset) ((swp_entry_t) { ((type) << 2) | ((offset) << 9) })
#define __pte_to_swp_entry(pte)	((swp_entry_t) { pte_val(pte) })
#define __swp_entry_to_pte(swp)	((pte_t) { (swp).val })

/* Needs to be defined here and not in linux/mm.h, as it is arch dependent */
/* FIXME: this is not correct */
#define kern_addr_valid(addr)	(1)

#include <asm-generic/pgtable.h>

#define HAVE_ARCH_UNMAPPED_AREA

#define io_remap_pfn_range(vma,from,pfn,size,prot) \
		remap_pfn_range(vma, from, pfn, size, prot)

#define MK_IOSPACE_PFN(space, pfn)	(pfn)
#define GET_IOSPACE(pfn)		0
#define GET_PFN(pfn)			(pfn)

#define pgtable_cache_init() do { } while (0)

#endif /* !__ASSEMBLY__ */

#endif /* CONFIG_MMU */

#endif /* _ASMARM_PGTABLE_H */
