

#include <linux/mm.h>

static unsigned long kernel_virtual_offset;
static int is_xencomm_initialized;

int
xencomm_is_initialized(void)
{
	return is_xencomm_initialized;
}

void
xencomm_initialize(void)
{
	kernel_virtual_offset = KERNEL_START - ia64_tpa(KERNEL_START);
	is_xencomm_initialized = 1;
}

/* Translate virtual address to physical address.  */
unsigned long
xencomm_vtop(unsigned long vaddr)
{
	struct page *page;
	struct vm_area_struct *vma;

	if (vaddr == 0)
		return 0UL;

	if (REGION_NUMBER(vaddr) == 5) {
		pgd_t *pgd;
		pud_t *pud;
		pmd_t *pmd;
		pte_t *ptep;

		/* On ia64, TASK_SIZE refers to current.  It is not initialized
		   during boot.
		   Furthermore the kernel is relocatable and __pa() doesn't
		   work on  addresses.  */
		if (vaddr >= KERNEL_START
		    && vaddr < (KERNEL_START + KERNEL_TR_PAGE_SIZE))
			return vaddr - kernel_virtual_offset;

		/* In kernel area -- virtually mapped.  */
		pgd = pgd_offset_k(vaddr);
		if (pgd_none(*pgd) || pgd_bad(*pgd))
			return ~0UL;

		pud = pud_offset(pgd, vaddr);
		if (pud_none(*pud) || pud_bad(*pud))
			return ~0UL;

		pmd = pmd_offset(pud, vaddr);
		if (pmd_none(*pmd) || pmd_bad(*pmd))
			return ~0UL;

		ptep = pte_offset_kernel(pmd, vaddr);
		if (!ptep)
			return ~0UL;

		return (pte_val(*ptep) & _PFN_MASK) | (vaddr & ~PAGE_MASK);
	}

	if (vaddr > TASK_SIZE) {
		/* percpu variables */
		if (REGION_NUMBER(vaddr) == 7 &&
		    REGION_OFFSET(vaddr) >= (1ULL << IA64_MAX_PHYS_BITS))
			ia64_tpa(vaddr);

		/* kernel address */
		return __pa(vaddr);
	}

	/* XXX double-check (lack of) locking */
	vma = find_extend_vma(current->mm, vaddr);
	if (!vma)
		return ~0UL;

	/* We assume the page is modified.  */
	page = follow_page(vma, vaddr, FOLL_WRITE | FOLL_TOUCH);
	if (!page)
		return ~0UL;

	return (page_to_pfn(page) << PAGE_SHIFT) | (vaddr & ~PAGE_MASK);
}
