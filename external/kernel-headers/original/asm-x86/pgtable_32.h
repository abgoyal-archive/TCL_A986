#ifndef _I386_PGTABLE_H
#define _I386_PGTABLE_H


#ifndef __ASSEMBLY__
#include <asm/processor.h>
#include <asm/fixmap.h>
#include <linux/threads.h>
#include <asm/paravirt.h>

#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/spinlock.h>

struct mm_struct;
struct vm_area_struct;

#define ZERO_PAGE(vaddr) (virt_to_page(empty_zero_page))
extern unsigned long empty_zero_page[1024];
extern pgd_t swapper_pg_dir[1024];
extern struct kmem_cache *pmd_cache;
extern spinlock_t pgd_lock;
extern struct page *pgd_list;
void check_pgt_cache(void);

void pmd_ctor(struct kmem_cache *, void *);
void pgtable_cache_init(void);
void paging_init(void);


#ifdef CONFIG_X86_PAE
# include <asm/pgtable-3level-defs.h>
# define PMD_SIZE	(1UL << PMD_SHIFT)
# define PMD_MASK	(~(PMD_SIZE-1))
#else
# include <asm/pgtable-2level-defs.h>
#endif

#define PGDIR_SIZE	(1UL << PGDIR_SHIFT)
#define PGDIR_MASK	(~(PGDIR_SIZE-1))

#define USER_PTRS_PER_PGD	(TASK_SIZE/PGDIR_SIZE)
#define FIRST_USER_ADDRESS	0

#define USER_PGD_PTRS (PAGE_OFFSET >> PGDIR_SHIFT)
#define KERNEL_PGD_PTRS (PTRS_PER_PGD-USER_PGD_PTRS)

#define TWOLEVEL_PGDIR_SHIFT	22
#define BOOT_USER_PGD_PTRS (__PAGE_OFFSET >> TWOLEVEL_PGDIR_SHIFT)
#define BOOT_KERNEL_PGD_PTRS (1024-BOOT_USER_PGD_PTRS)

#define VMALLOC_OFFSET	(8*1024*1024)
#define VMALLOC_START	(((unsigned long) high_memory + \
			2*VMALLOC_OFFSET-1) & ~(VMALLOC_OFFSET-1))
#ifdef CONFIG_HIGHMEM
# define VMALLOC_END	(PKMAP_BASE-2*PAGE_SIZE)
#else
# define VMALLOC_END	(FIXADDR_START-2*PAGE_SIZE)
#endif

#define _PAGE_BIT_PRESENT	0
#define _PAGE_BIT_RW		1
#define _PAGE_BIT_USER		2
#define _PAGE_BIT_PWT		3
#define _PAGE_BIT_PCD		4
#define _PAGE_BIT_ACCESSED	5
#define _PAGE_BIT_DIRTY		6
#define _PAGE_BIT_PSE		7	/* 4 MB (or 2MB) page, Pentium+, if present.. */
#define _PAGE_BIT_GLOBAL	8	/* Global TLB entry PPro+ */
#define _PAGE_BIT_UNUSED1	9	/* available for programmer */
#define _PAGE_BIT_UNUSED2	10
#define _PAGE_BIT_UNUSED3	11
#define _PAGE_BIT_NX		63

#define _PAGE_PRESENT	0x001
#define _PAGE_RW	0x002
#define _PAGE_USER	0x004
#define _PAGE_PWT	0x008
#define _PAGE_PCD	0x010
#define _PAGE_ACCESSED	0x020
#define _PAGE_DIRTY	0x040
#define _PAGE_PSE	0x080	/* 4 MB (or 2MB) page, Pentium+, if present.. */
#define _PAGE_GLOBAL	0x100	/* Global TLB entry PPro+ */
#define _PAGE_UNUSED1	0x200	/* available for programmer */
#define _PAGE_UNUSED2	0x400
#define _PAGE_UNUSED3	0x800

/* If _PAGE_PRESENT is clear, we use these: */
#define _PAGE_FILE	0x040	/* nonlinear file mapping, saved PTE; unset:swap */
#define _PAGE_PROTNONE	0x080	/* if the user mapped it with PROT_NONE;
				   pte_present gives true */
#ifdef CONFIG_X86_PAE
#define _PAGE_NX	(1ULL<<_PAGE_BIT_NX)
#else
#define _PAGE_NX	0
#endif

#define _PAGE_TABLE	(_PAGE_PRESENT | _PAGE_RW | _PAGE_USER | _PAGE_ACCESSED | _PAGE_DIRTY)
#define _KERNPG_TABLE	(_PAGE_PRESENT | _PAGE_RW | _PAGE_ACCESSED | _PAGE_DIRTY)
#define _PAGE_CHG_MASK	(PTE_MASK | _PAGE_ACCESSED | _PAGE_DIRTY)

#define PAGE_NONE \
	__pgprot(_PAGE_PROTNONE | _PAGE_ACCESSED)
#define PAGE_SHARED \
	__pgprot(_PAGE_PRESENT | _PAGE_RW | _PAGE_USER | _PAGE_ACCESSED)

#define PAGE_SHARED_EXEC \
	__pgprot(_PAGE_PRESENT | _PAGE_RW | _PAGE_USER | _PAGE_ACCESSED)
#define PAGE_COPY_NOEXEC \
	__pgprot(_PAGE_PRESENT | _PAGE_USER | _PAGE_ACCESSED | _PAGE_NX)
#define PAGE_COPY_EXEC \
	__pgprot(_PAGE_PRESENT | _PAGE_USER | _PAGE_ACCESSED)
#define PAGE_COPY \
	PAGE_COPY_NOEXEC
#define PAGE_READONLY \
	__pgprot(_PAGE_PRESENT | _PAGE_USER | _PAGE_ACCESSED | _PAGE_NX)
#define PAGE_READONLY_EXEC \
	__pgprot(_PAGE_PRESENT | _PAGE_USER | _PAGE_ACCESSED)

#define _PAGE_KERNEL \
	(_PAGE_PRESENT | _PAGE_RW | _PAGE_DIRTY | _PAGE_ACCESSED | _PAGE_NX)
#define _PAGE_KERNEL_EXEC \
	(_PAGE_PRESENT | _PAGE_RW | _PAGE_DIRTY | _PAGE_ACCESSED)

extern unsigned long long __PAGE_KERNEL, __PAGE_KERNEL_EXEC;
#define __PAGE_KERNEL_RO		(__PAGE_KERNEL & ~_PAGE_RW)
#define __PAGE_KERNEL_RX		(__PAGE_KERNEL_EXEC & ~_PAGE_RW)
#define __PAGE_KERNEL_NOCACHE		(__PAGE_KERNEL | _PAGE_PCD)
#define __PAGE_KERNEL_LARGE		(__PAGE_KERNEL | _PAGE_PSE)
#define __PAGE_KERNEL_LARGE_EXEC	(__PAGE_KERNEL_EXEC | _PAGE_PSE)

#define PAGE_KERNEL		__pgprot(__PAGE_KERNEL)
#define PAGE_KERNEL_RO		__pgprot(__PAGE_KERNEL_RO)
#define PAGE_KERNEL_EXEC	__pgprot(__PAGE_KERNEL_EXEC)
#define PAGE_KERNEL_RX		__pgprot(__PAGE_KERNEL_RX)
#define PAGE_KERNEL_NOCACHE	__pgprot(__PAGE_KERNEL_NOCACHE)
#define PAGE_KERNEL_LARGE	__pgprot(__PAGE_KERNEL_LARGE)
#define PAGE_KERNEL_LARGE_EXEC	__pgprot(__PAGE_KERNEL_LARGE_EXEC)

#define __P000	PAGE_NONE
#define __P001	PAGE_READONLY
#define __P010	PAGE_COPY
#define __P011	PAGE_COPY
#define __P100	PAGE_READONLY_EXEC
#define __P101	PAGE_READONLY_EXEC
#define __P110	PAGE_COPY_EXEC
#define __P111	PAGE_COPY_EXEC

#define __S000	PAGE_NONE
#define __S001	PAGE_READONLY
#define __S010	PAGE_SHARED
#define __S011	PAGE_SHARED
#define __S100	PAGE_READONLY_EXEC
#define __S101	PAGE_READONLY_EXEC
#define __S110	PAGE_SHARED_EXEC
#define __S111	PAGE_SHARED_EXEC

#undef TEST_ACCESS_OK

/* The boot page tables (all created as a single array) */
extern unsigned long pg0[];

#define pte_present(x)	((x).pte_low & (_PAGE_PRESENT | _PAGE_PROTNONE))

/* To avoid harmful races, pmd_none(x) should check only the lower when PAE */
#define pmd_none(x)	(!(unsigned long)pmd_val(x))
#define pmd_present(x)	(pmd_val(x) & _PAGE_PRESENT)
#define	pmd_bad(x)	((pmd_val(x) & (~PAGE_MASK & ~_PAGE_USER)) != _KERNPG_TABLE)


#define pages_to_mb(x) ((x) >> (20-PAGE_SHIFT))

static inline int pte_dirty(pte_t pte)		{ return (pte).pte_low & _PAGE_DIRTY; }
static inline int pte_young(pte_t pte)		{ return (pte).pte_low & _PAGE_ACCESSED; }
static inline int pte_write(pte_t pte)		{ return (pte).pte_low & _PAGE_RW; }
static inline int pte_huge(pte_t pte)		{ return (pte).pte_low & _PAGE_PSE; }

static inline int pte_file(pte_t pte)		{ return (pte).pte_low & _PAGE_FILE; }

static inline pte_t pte_mkclean(pte_t pte)	{ (pte).pte_low &= ~_PAGE_DIRTY; return pte; }
static inline pte_t pte_mkold(pte_t pte)	{ (pte).pte_low &= ~_PAGE_ACCESSED; return pte; }
static inline pte_t pte_wrprotect(pte_t pte)	{ (pte).pte_low &= ~_PAGE_RW; return pte; }
static inline pte_t pte_mkdirty(pte_t pte)	{ (pte).pte_low |= _PAGE_DIRTY; return pte; }
static inline pte_t pte_mkyoung(pte_t pte)	{ (pte).pte_low |= _PAGE_ACCESSED; return pte; }
static inline pte_t pte_mkwrite(pte_t pte)	{ (pte).pte_low |= _PAGE_RW; return pte; }
static inline pte_t pte_mkhuge(pte_t pte)	{ (pte).pte_low |= _PAGE_PSE; return pte; }

#ifdef CONFIG_X86_PAE
# include <asm/pgtable-3level.h>
#else
# include <asm/pgtable-2level.h>
#endif

#ifndef CONFIG_PARAVIRT
#define pte_update(mm, addr, ptep)		do { } while (0)
#define pte_update_defer(mm, addr, ptep)	do { } while (0)
#endif

/* local pte updates need not use xchg for locking */
static inline pte_t native_local_ptep_get_and_clear(pte_t *ptep)
{
	pte_t res = *ptep;

	/* Pure native function needs no input for mm, addr */
	native_pte_clear(NULL, 0, ptep);
	return res;
}

#define  __HAVE_ARCH_PTEP_SET_ACCESS_FLAGS
#define ptep_set_access_flags(vma, address, ptep, entry, dirty)		\
({									\
	int __changed = !pte_same(*(ptep), entry);			\
	if (__changed && dirty) {					\
		(ptep)->pte_low = (entry).pte_low;			\
		pte_update_defer((vma)->vm_mm, (address), (ptep));	\
		flush_tlb_page(vma, address);				\
	}								\
	__changed;							\
})

#define __HAVE_ARCH_PTEP_TEST_AND_CLEAR_YOUNG
#define ptep_test_and_clear_young(vma, addr, ptep) ({			\
	int __ret = 0;							\
	if (pte_young(*(ptep)))						\
		__ret = test_and_clear_bit(_PAGE_BIT_ACCESSED,		\
						&(ptep)->pte_low);	\
	if (__ret)							\
		pte_update((vma)->vm_mm, addr, ptep);			\
	__ret;								\
})

#define __HAVE_ARCH_PTEP_CLEAR_YOUNG_FLUSH
#define ptep_clear_flush_young(vma, address, ptep)			\
({									\
	int __young;							\
	__young = ptep_test_and_clear_young((vma), (address), (ptep));	\
	if (__young)							\
		flush_tlb_page(vma, address);				\
	__young;							\
})

#define __HAVE_ARCH_PTEP_GET_AND_CLEAR
static inline pte_t ptep_get_and_clear(struct mm_struct *mm, unsigned long addr, pte_t *ptep)
{
	pte_t pte = native_ptep_get_and_clear(ptep);
	pte_update(mm, addr, ptep);
	return pte;
}

#define __HAVE_ARCH_PTEP_GET_AND_CLEAR_FULL
static inline pte_t ptep_get_and_clear_full(struct mm_struct *mm, unsigned long addr, pte_t *ptep, int full)
{
	pte_t pte;
	if (full) {
		/*
		 * Full address destruction in progress; paravirt does not
		 * care about updates and native needs no locking
		 */
		pte = native_local_ptep_get_and_clear(ptep);
	} else {
		pte = ptep_get_and_clear(mm, addr, ptep);
	}
	return pte;
}

#define __HAVE_ARCH_PTEP_SET_WRPROTECT
static inline void ptep_set_wrprotect(struct mm_struct *mm, unsigned long addr, pte_t *ptep)
{
	clear_bit(_PAGE_BIT_RW, &ptep->pte_low);
	pte_update(mm, addr, ptep);
}

static inline void clone_pgd_range(pgd_t *dst, pgd_t *src, int count)
{
       memcpy(dst, src, count * sizeof(pgd_t));
}

#define pgprot_noncached(prot)	((boot_cpu_data.x86 > 3)					  \
				 ? (__pgprot(pgprot_val(prot) | _PAGE_PCD | _PAGE_PWT)) : (prot))


#define mk_pte(page, pgprot)	pfn_pte(page_to_pfn(page), (pgprot))

static inline pte_t pte_modify(pte_t pte, pgprot_t newprot)
{
	pte.pte_low &= _PAGE_CHG_MASK;
	pte.pte_low |= pgprot_val(newprot);
#ifdef CONFIG_X86_PAE
	/*
	 * Chop off the NX bit (if present), and add the NX portion of
	 * the newprot (if present):
	 */
	pte.pte_high &= ~(1 << (_PAGE_BIT_NX - 32));
	pte.pte_high |= (pgprot_val(newprot) >> 32) & \
					(__supported_pte_mask >> 32);
#endif
	return pte;
}

#define pmd_large(pmd) \
((pmd_val(pmd) & (_PAGE_PSE|_PAGE_PRESENT)) == (_PAGE_PSE|_PAGE_PRESENT))

#define pgd_index(address) (((address) >> PGDIR_SHIFT) & (PTRS_PER_PGD-1))
#define pgd_index_k(addr) pgd_index(addr)

#define pgd_offset(mm, address) ((mm)->pgd+pgd_index(address))

#define pgd_offset_k(address) pgd_offset(&init_mm, address)

#define pmd_index(address) \
		(((address) >> PMD_SHIFT) & (PTRS_PER_PMD-1))

#define pte_index(address) \
		(((address) >> PAGE_SHIFT) & (PTRS_PER_PTE - 1))
#define pte_offset_kernel(dir, address) \
	((pte_t *) pmd_page_vaddr(*(dir)) +  pte_index(address))

#define pmd_page(pmd) (pfn_to_page(pmd_val(pmd) >> PAGE_SHIFT))

#define pmd_page_vaddr(pmd) \
		((unsigned long) __va(pmd_val(pmd) & PAGE_MASK))

extern pte_t *lookup_address(unsigned long address);

#ifdef CONFIG_X86_PAE
 extern int set_kernel_exec(unsigned long vaddr, int enable);
#else
 static inline int set_kernel_exec(unsigned long vaddr, int enable) { return 0;}
#endif

#if defined(CONFIG_HIGHPTE)
#define pte_offset_map(dir, address) \
	((pte_t *)kmap_atomic_pte(pmd_page(*(dir)),KM_PTE0) + pte_index(address))
#define pte_offset_map_nested(dir, address) \
	((pte_t *)kmap_atomic_pte(pmd_page(*(dir)),KM_PTE1) + pte_index(address))
#define pte_unmap(pte) kunmap_atomic(pte, KM_PTE0)
#define pte_unmap_nested(pte) kunmap_atomic(pte, KM_PTE1)
#else
#define pte_offset_map(dir, address) \
	((pte_t *)page_address(pmd_page(*(dir))) + pte_index(address))
#define pte_offset_map_nested(dir, address) pte_offset_map(dir, address)
#define pte_unmap(pte) do { } while (0)
#define pte_unmap_nested(pte) do { } while (0)
#endif

/* Clear a kernel PTE and flush it from the TLB */
#define kpte_clear_flush(ptep, vaddr)					\
do {									\
	pte_clear(&init_mm, vaddr, ptep);				\
	__flush_tlb_one(vaddr);						\
} while (0)

#define update_mmu_cache(vma,address,pte) do { } while (0)

void native_pagetable_setup_start(pgd_t *base);
void native_pagetable_setup_done(pgd_t *base);

#ifndef CONFIG_PARAVIRT
static inline void paravirt_pagetable_setup_start(pgd_t *base)
{
	native_pagetable_setup_start(base);
}

static inline void paravirt_pagetable_setup_done(pgd_t *base)
{
	native_pagetable_setup_done(base);
}
#endif	/* !CONFIG_PARAVIRT */

#endif /* !__ASSEMBLY__ */

#ifdef CONFIG_FLATMEM
#define kern_addr_valid(addr)	(1)
#endif /* CONFIG_FLATMEM */

#define io_remap_pfn_range(vma, vaddr, pfn, size, prot)		\
		remap_pfn_range(vma, vaddr, pfn, size, prot)

#include <asm-generic/pgtable.h>

#endif /* _I386_PGTABLE_H */
