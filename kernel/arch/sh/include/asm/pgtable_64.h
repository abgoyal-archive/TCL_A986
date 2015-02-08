
#ifndef __ASM_SH_PGTABLE_64_H
#define __ASM_SH_PGTABLE_64_H

#include <linux/threads.h>
#include <asm/processor.h>
#include <asm/page.h>

#define pte_ERROR(e) \
	printk("%s:%d: bad pte %016Lx.\n", __FILE__, __LINE__, pte_val(e))
#define pgd_ERROR(e) \
	printk("%s:%d: bad pgd %08lx.\n", __FILE__, __LINE__, pgd_val(e))

#define set_pmd(pmdptr, pmdval) (*(pmdptr) = pmdval)

static __inline__ void set_pte(pte_t *pteptr, pte_t pteval)
{
	unsigned long long x = ((unsigned long long) pteval.pte_low);
	unsigned long long *xp = (unsigned long long *) pteptr;
	/*
	 * Sign-extend based on NPHYS.
	 */
	*(xp) = (x & NPHYS_SIGN) ? (x | NPHYS_MASK) : x;
}
#define set_pte_at(mm,addr,ptep,pteval) set_pte(ptep,pteval)


/* To find an entry in a generic PGD. */
#define pgd_index(address) (((address) >> PGDIR_SHIFT) & (PTRS_PER_PGD-1))
#define __pgd_offset(address) pgd_index(address)
#define pgd_offset(mm, address) ((mm)->pgd+pgd_index(address))

/* To find an entry in a kernel PGD. */
#define pgd_offset_k(address) pgd_offset(&init_mm, address)

#define __pud_offset(address)	(((address) >> PUD_SHIFT) & (PTRS_PER_PUD-1))
#define __pmd_offset(address)	(((address) >> PMD_SHIFT) & (PTRS_PER_PMD-1))

#define _PMD_EMPTY		0x0
/* Either the PMD is empty or present, it's not paged out */
#define pmd_present(pmd_entry)	(pmd_val(pmd_entry) & _PAGE_PRESENT)
#define pmd_clear(pmd_entry_p)	(set_pmd((pmd_entry_p), __pmd(_PMD_EMPTY)))
#define pmd_none(pmd_entry)	(pmd_val((pmd_entry)) == _PMD_EMPTY)
#define pmd_bad(pmd_entry)	((pmd_val(pmd_entry) & (~PAGE_MASK & ~_PAGE_USER)) != _KERNPG_TABLE)

#define pmd_page_vaddr(pmd_entry) \
	((unsigned long) __va(pmd_val(pmd_entry) & PAGE_MASK))

#define pmd_page(pmd) \
	(virt_to_page(pmd_val(pmd)))

/* PMD to PTE dereferencing */
#define pte_index(address) \
		((address >> PAGE_SHIFT) & (PTRS_PER_PTE - 1))

#define __pte_offset(address)	pte_index(address)

#define pte_offset_kernel(dir, addr) \
		((pte_t *) ((pmd_val(*(dir))) & PAGE_MASK) + pte_index((addr)))

#define pte_offset_map(dir,addr)	pte_offset_kernel(dir, addr)
#define pte_offset_map_nested(dir,addr)	pte_offset_kernel(dir, addr)
#define pte_unmap(pte)		do { } while (0)
#define pte_unmap_nested(pte)	do { } while (0)

#ifndef __ASSEMBLY__
#define IOBASE_VADDR	0xff000000
#define IOBASE_END	0xffffffff

#define _PAGE_WT	0x001  /* CB0: if cacheable, 1->write-thru, 0->write-back */
#define _PAGE_DEVICE	0x001  /* CB0: if uncacheable, 1->device (i.e. no write-combining or reordering at bus level) */
#define _PAGE_CACHABLE	0x002  /* CB1: uncachable/cachable */
#define _PAGE_PRESENT	0x004  /* software: page referenced */
#define _PAGE_FILE	0x004  /* software: only when !present */
#define _PAGE_SIZE0	0x008  /* SZ0-bit : size of page */
#define _PAGE_SIZE1	0x010  /* SZ1-bit : size of page */
#define _PAGE_SHARED	0x020  /* software: reflects PTEH's SH */
#define _PAGE_READ	0x040  /* PR0-bit : read access allowed */
#define _PAGE_EXECUTE	0x080  /* PR1-bit : execute access allowed */
#define _PAGE_WRITE	0x100  /* PR2-bit : write access allowed */
#define _PAGE_USER	0x200  /* PR3-bit : user space access allowed */
#define _PAGE_DIRTY	0x400  /* software: page accessed in write */
#define _PAGE_ACCESSED	0x800  /* software: page referenced */

/* Wrapper for extended mode pgprot twiddling */
#define _PAGE_EXT(x)		((unsigned long long)(x) << 32)

#define _PAGE_WIRED	_PAGE_EXT(0x001) /* software: wire the tlb entry */

#define _PAGE_CLEAR_FLAGS	(_PAGE_PRESENT | _PAGE_FILE | _PAGE_SHARED | \
				 _PAGE_DIRTY | _PAGE_ACCESSED | _PAGE_WIRED)

/* Mask which drops software flags */
#define _PAGE_FLAGS_HARDWARE_MASK	(NEFF_MASK & ~(_PAGE_CLEAR_FLAGS))

#if defined(CONFIG_HUGETLB_PAGE_SIZE_64K)
#define _PAGE_SZHUGE	(_PAGE_SIZE0)
#elif defined(CONFIG_HUGETLB_PAGE_SIZE_1MB)
#define _PAGE_SZHUGE	(_PAGE_SIZE1)
#elif defined(CONFIG_HUGETLB_PAGE_SIZE_512MB)
#define _PAGE_SZHUGE	(_PAGE_SIZE0 | _PAGE_SIZE1)
#endif

#ifndef _PAGE_SZHUGE
# define _PAGE_SZHUGE	(0)
#endif

#define _KERNPG_TABLE	(_PAGE_PRESENT | _PAGE_READ | _PAGE_WRITE | \
			 _PAGE_EXECUTE | \
			 _PAGE_CACHABLE | _PAGE_ACCESSED | _PAGE_DIRTY | \
			 _PAGE_SHARED)

/* Default flags for a User page */
#define _PAGE_TABLE	(_KERNPG_TABLE | _PAGE_USER)

#define _PAGE_CHG_MASK	(PTE_MASK | _PAGE_ACCESSED | _PAGE_DIRTY)

#define _PAGE_COMMON	(_PAGE_PRESENT | _PAGE_USER | \
			 _PAGE_CACHABLE | _PAGE_ACCESSED)

#define PAGE_NONE	__pgprot(_PAGE_CACHABLE | _PAGE_ACCESSED)
#define PAGE_SHARED	__pgprot(_PAGE_COMMON | _PAGE_READ | _PAGE_WRITE | \
				 _PAGE_SHARED)
#define PAGE_EXECREAD	__pgprot(_PAGE_COMMON | _PAGE_READ | _PAGE_EXECUTE)

#define PAGE_COPY	PAGE_EXECREAD

#define PAGE_READONLY	__pgprot(_PAGE_COMMON | _PAGE_READ)
#define PAGE_WRITEONLY	__pgprot(_PAGE_COMMON | _PAGE_WRITE)
#define PAGE_RWX	__pgprot(_PAGE_COMMON | _PAGE_READ | \
				 _PAGE_WRITE | _PAGE_EXECUTE)
#define PAGE_KERNEL	__pgprot(_KERNPG_TABLE)

#define PAGE_KERNEL_NOCACHE \
			__pgprot(_PAGE_PRESENT | _PAGE_READ | _PAGE_WRITE | \
				 _PAGE_EXECUTE | _PAGE_ACCESSED | \
				 _PAGE_DIRTY | _PAGE_SHARED)

#define pgprot_noncached(x) __pgprot(((x).pgprot & ~(_PAGE_CACHABLE)) | _PAGE_DEVICE)
#define pgprot_writecombine(prot) __pgprot(pgprot_val(prot) & ~_PAGE_CACHABLE)

#define _PTE_EMPTY	0x0
#define pte_present(x)	(pte_val(x) & _PAGE_PRESENT)
#define pte_clear(mm,addr,xp)	(set_pte_at(mm, addr, xp, __pte(_PTE_EMPTY)))
#define pte_none(x)	(pte_val(x) == _PTE_EMPTY)


#define pte_pagenr(x)		(((unsigned long) (pte_val(x)) - \
				 __MEMORY_START) >> PAGE_SHIFT)

#define pte_page(x)		(mem_map + pte_pagenr(x))

#define pages_to_mb(x) ((x) >> (20-PAGE_SHIFT))


static inline int pte_dirty(pte_t pte)  { return pte_val(pte) & _PAGE_DIRTY; }
static inline int pte_young(pte_t pte)  { return pte_val(pte) & _PAGE_ACCESSED; }
static inline int pte_file(pte_t pte)   { return pte_val(pte) & _PAGE_FILE; }
static inline int pte_write(pte_t pte)  { return pte_val(pte) & _PAGE_WRITE; }
static inline int pte_special(pte_t pte){ return 0; }

static inline pte_t pte_wrprotect(pte_t pte)	{ set_pte(&pte, __pte(pte_val(pte) & ~_PAGE_WRITE)); return pte; }
static inline pte_t pte_mkclean(pte_t pte)	{ set_pte(&pte, __pte(pte_val(pte) & ~_PAGE_DIRTY)); return pte; }
static inline pte_t pte_mkold(pte_t pte)	{ set_pte(&pte, __pte(pte_val(pte) & ~_PAGE_ACCESSED)); return pte; }
static inline pte_t pte_mkwrite(pte_t pte)	{ set_pte(&pte, __pte(pte_val(pte) | _PAGE_WRITE)); return pte; }
static inline pte_t pte_mkdirty(pte_t pte)	{ set_pte(&pte, __pte(pte_val(pte) | _PAGE_DIRTY)); return pte; }
static inline pte_t pte_mkyoung(pte_t pte)	{ set_pte(&pte, __pte(pte_val(pte) | _PAGE_ACCESSED)); return pte; }
static inline pte_t pte_mkhuge(pte_t pte)	{ set_pte(&pte, __pte(pte_val(pte) | _PAGE_SZHUGE)); return pte; }
static inline pte_t pte_mkspecial(pte_t pte)	{ return pte; }


#define mk_pte(page,pgprot)							\
({										\
	pte_t __pte;								\
										\
	set_pte(&__pte, __pte((((page)-mem_map) << PAGE_SHIFT) | 		\
		__MEMORY_START | pgprot_val((pgprot))));			\
	__pte;									\
})

#define mk_pte_phys(physpage, pgprot) \
({ pte_t __pte; set_pte(&__pte, __pte(physpage | pgprot_val(pgprot))); __pte; })

static inline pte_t pte_modify(pte_t pte, pgprot_t newprot)
{ set_pte(&pte, __pte((pte_val(pte) & _PAGE_CHG_MASK) | pgprot_val(newprot))); return pte; }

/* Encode and decode a swap entry */
#define __swp_type(x)			(((x).val & 3) + (((x).val >> 1) & 0x3c))
#define __swp_offset(x)			((x).val >> 8)
#define __swp_entry(type, offset)	((swp_entry_t) { ((offset << 8) + ((type & 0x3c) << 1) + (type & 3)) })
#define __pte_to_swp_entry(pte)		((swp_entry_t) { pte_val(pte) })
#define __swp_entry_to_pte(x)		((pte_t) { (x).val })

/* Encode and decode a nonlinear file mapping entry */
#define PTE_FILE_MAX_BITS		29
#define pte_to_pgoff(pte)		(pte_val(pte))
#define pgoff_to_pte(off)		((pte_t) { (off) | _PAGE_FILE })

#endif /* !__ASSEMBLY__ */

#define pfn_pte(pfn, prot)	__pte(((pfn) << PAGE_SHIFT) | pgprot_val(prot))
#define pfn_pmd(pfn, prot)	__pmd(((pfn) << PAGE_SHIFT) | pgprot_val(prot))

#endif /* __ASM_SH_PGTABLE_64_H */
