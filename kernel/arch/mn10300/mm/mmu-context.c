
#include <linux/sched.h>
#include <linux/mm.h>
#include <asm/mmu_context.h>
#include <asm/tlbflush.h>

unsigned long mmu_context_cache[NR_CPUS] = {
	[0 ... NR_CPUS - 1] = MMU_CONTEXT_FIRST_VERSION * 2 - 1,
};

void flush_tlb_page(struct vm_area_struct *vma, unsigned long addr)
{
	unsigned long pteu, cnx, flags;

	addr &= PAGE_MASK;

	/* make sure the context doesn't migrate and defend against
	 * interference from vmalloc'd regions */
	local_irq_save(flags);

	cnx = mm_context(vma->vm_mm);

	if (cnx != MMU_NO_CONTEXT) {
		pteu = addr | (cnx & 0x000000ffUL);
		IPTEU = pteu;
		DPTEU = pteu;
		if (IPTEL & xPTEL_V)
			IPTEL = 0;
		if (DPTEL & xPTEL_V)
			DPTEL = 0;
	}

	local_irq_restore(flags);
}

void update_mmu_cache(struct vm_area_struct *vma, unsigned long addr, pte_t *ptep)
{
	unsigned long pteu, ptel, cnx, flags;
	pte_t pte = *ptep;

	addr &= PAGE_MASK;
	ptel = pte_val(pte) & ~(xPTEL_UNUSED1 | xPTEL_UNUSED2);

	/* make sure the context doesn't migrate and defend against
	 * interference from vmalloc'd regions */
	local_irq_save(flags);

	cnx = mm_context(vma->vm_mm);

	if (cnx != MMU_NO_CONTEXT) {
		pteu = addr | (cnx & 0x000000ffUL);
		if (!(pte_val(pte) & _PAGE_NX)) {
			IPTEU = pteu;
			if (IPTEL & xPTEL_V)
				IPTEL = ptel;
		}
		DPTEU = pteu;
		if (DPTEL & xPTEL_V)
			DPTEL = ptel;
	}

	local_irq_restore(flags);
}
