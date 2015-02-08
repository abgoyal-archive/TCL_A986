

#include <linux/kernel.h>
#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/percpu.h>
#include <linux/hardirq.h>
#include <asm/pgalloc.h>
#include <asm/tlbflush.h>
#include <asm/tlb.h>

#include "mmu_decl.h"

DEFINE_PER_CPU(struct mmu_gather, mmu_gathers);

#ifdef CONFIG_SMP


static DEFINE_PER_CPU(struct pte_freelist_batch *, pte_freelist_cur);
static unsigned long pte_freelist_forced_free;

struct pte_freelist_batch
{
	struct rcu_head	rcu;
	unsigned int	index;
	unsigned long	tables[0];
};

#define PTE_FREELIST_SIZE \
	((PAGE_SIZE - sizeof(struct pte_freelist_batch)) \
	  / sizeof(unsigned long))

static void pte_free_smp_sync(void *arg)
{
	/* Do nothing, just ensure we sync with all CPUs */
}

static void pgtable_free_now(void *table, unsigned shift)
{
	pte_freelist_forced_free++;

	smp_call_function(pte_free_smp_sync, NULL, 1);

	pgtable_free(table, shift);
}

static void pte_free_rcu_callback(struct rcu_head *head)
{
	struct pte_freelist_batch *batch =
		container_of(head, struct pte_freelist_batch, rcu);
	unsigned int i;

	for (i = 0; i < batch->index; i++) {
		void *table = (void *)(batch->tables[i] & ~MAX_PGTABLE_INDEX_SIZE);
		unsigned shift = batch->tables[i] & MAX_PGTABLE_INDEX_SIZE;

		pgtable_free(table, shift);
	}

	free_page((unsigned long)batch);
}

static void pte_free_submit(struct pte_freelist_batch *batch)
{
	INIT_RCU_HEAD(&batch->rcu);
	call_rcu(&batch->rcu, pte_free_rcu_callback);
}

void pgtable_free_tlb(struct mmu_gather *tlb, void *table, unsigned shift)
{
	/* This is safe since tlb_gather_mmu has disabled preemption */
	struct pte_freelist_batch **batchp = &__get_cpu_var(pte_freelist_cur);
	unsigned long pgf;

	if (atomic_read(&tlb->mm->mm_users) < 2 ||
	    cpumask_equal(mm_cpumask(tlb->mm), cpumask_of(smp_processor_id()))){
		pgtable_free(table, shift);
		return;
	}

	if (*batchp == NULL) {
		*batchp = (struct pte_freelist_batch *)__get_free_page(GFP_ATOMIC);
		if (*batchp == NULL) {
			pgtable_free_now(table, shift);
			return;
		}
		(*batchp)->index = 0;
	}
	BUG_ON(shift > MAX_PGTABLE_INDEX_SIZE);
	pgf = (unsigned long)table | shift;
	(*batchp)->tables[(*batchp)->index++] = pgf;
	if ((*batchp)->index == PTE_FREELIST_SIZE) {
		pte_free_submit(*batchp);
		*batchp = NULL;
	}
}

void pte_free_finish(void)
{
	/* This is safe since tlb_gather_mmu has disabled preemption */
	struct pte_freelist_batch **batchp = &__get_cpu_var(pte_freelist_cur);

	if (*batchp == NULL)
		return;
	pte_free_submit(*batchp);
	*batchp = NULL;
}

#endif /* CONFIG_SMP */

static inline int is_exec_fault(void)
{
	return current->thread.regs && TRAP(current->thread.regs) == 0x400;
}

static inline int pte_looks_normal(pte_t pte)
{
	return (pte_val(pte) &
	    (_PAGE_PRESENT | _PAGE_SPECIAL | _PAGE_NO_CACHE | _PAGE_USER)) ==
	    (_PAGE_PRESENT | _PAGE_USER);
}

struct page * maybe_pte_to_page(pte_t pte)
{
	unsigned long pfn = pte_pfn(pte);
	struct page *page;

	if (unlikely(!pfn_valid(pfn)))
		return NULL;
	page = pfn_to_page(pfn);
	if (PageReserved(page))
		return NULL;
	return page;
}

#if defined(CONFIG_PPC_STD_MMU) || _PAGE_EXEC == 0


static pte_t set_pte_filter(pte_t pte, unsigned long addr)
{
	pte = __pte(pte_val(pte) & ~_PAGE_HPTEFLAGS);
	if (pte_looks_normal(pte) && !(cpu_has_feature(CPU_FTR_COHERENT_ICACHE) ||
				       cpu_has_feature(CPU_FTR_NOEXECUTE))) {
		struct page *pg = maybe_pte_to_page(pte);
		if (!pg)
			return pte;
		if (!test_bit(PG_arch_1, &pg->flags)) {
#ifdef CONFIG_8xx
			/* On 8xx, cache control instructions (particularly
			 * "dcbst" from flush_dcache_icache) fault as write
			 * operation if there is an unpopulated TLB entry
			 * for the address in question. To workaround that,
			 * we invalidate the TLB here, thus avoiding dcbst
			 * misbehaviour.
			 */
			/* 8xx doesn't care about PID, size or ind args */
			_tlbil_va(addr, 0, 0, 0);
#endif /* CONFIG_8xx */
			flush_dcache_icache_page(pg);
			set_bit(PG_arch_1, &pg->flags);
		}
	}
	return pte;
}

static pte_t set_access_flags_filter(pte_t pte, struct vm_area_struct *vma,
				     int dirty)
{
	return pte;
}

#else /* defined(CONFIG_PPC_STD_MMU) || _PAGE_EXEC == 0 */

static pte_t set_pte_filter(pte_t pte, unsigned long addr)
{
	struct page *pg;

	/* No exec permission in the first place, move on */
	if (!(pte_val(pte) & _PAGE_EXEC) || !pte_looks_normal(pte))
		return pte;

	/* If you set _PAGE_EXEC on weird pages you're on your own */
	pg = maybe_pte_to_page(pte);
	if (unlikely(!pg))
		return pte;

	/* If the page clean, we move on */
	if (test_bit(PG_arch_1, &pg->flags))
		return pte;

	/* If it's an exec fault, we flush the cache and make it clean */
	if (is_exec_fault()) {
		flush_dcache_icache_page(pg);
		set_bit(PG_arch_1, &pg->flags);
		return pte;
	}

	/* Else, we filter out _PAGE_EXEC */
	return __pte(pte_val(pte) & ~_PAGE_EXEC);
}

static pte_t set_access_flags_filter(pte_t pte, struct vm_area_struct *vma,
				     int dirty)
{
	struct page *pg;

	/* So here, we only care about exec faults, as we use them
	 * to recover lost _PAGE_EXEC and perform I$/D$ coherency
	 * if necessary. Also if _PAGE_EXEC is already set, same deal,
	 * we just bail out
	 */
	if (dirty || (pte_val(pte) & _PAGE_EXEC) || !is_exec_fault())
		return pte;

#ifdef CONFIG_DEBUG_VM
	/* So this is an exec fault, _PAGE_EXEC is not set. If it was
	 * an error we would have bailed out earlier in do_page_fault()
	 * but let's make sure of it
	 */
	if (WARN_ON(!(vma->vm_flags & VM_EXEC)))
		return pte;
#endif /* CONFIG_DEBUG_VM */

	/* If you set _PAGE_EXEC on weird pages you're on your own */
	pg = maybe_pte_to_page(pte);
	if (unlikely(!pg))
		goto bail;

	/* If the page is already clean, we move on */
	if (test_bit(PG_arch_1, &pg->flags))
		goto bail;

	/* Clean the page and set PG_arch_1 */
	flush_dcache_icache_page(pg);
	set_bit(PG_arch_1, &pg->flags);

 bail:
	return __pte(pte_val(pte) | _PAGE_EXEC);
}

#endif /* !(defined(CONFIG_PPC_STD_MMU) || _PAGE_EXEC == 0) */

void set_pte_at(struct mm_struct *mm, unsigned long addr, pte_t *ptep,
		pte_t pte)
{
#ifdef CONFIG_DEBUG_VM
	WARN_ON(pte_present(*ptep));
#endif
	/* Note: mm->context.id might not yet have been assigned as
	 * this context might not have been activated yet when this
	 * is called.
	 */
	pte = set_pte_filter(pte, addr);

	/* Perform the setting of the PTE */
	__set_pte_at(mm, addr, ptep, pte, 0);
}

int ptep_set_access_flags(struct vm_area_struct *vma, unsigned long address,
			  pte_t *ptep, pte_t entry, int dirty)
{
	int changed;
	entry = set_access_flags_filter(entry, vma, dirty);
	changed = !pte_same(*(ptep), entry);
	if (changed) {
		if (!(vma->vm_flags & VM_HUGETLB))
			assert_pte_locked(vma->vm_mm, address);
		__ptep_set_access_flags(ptep, entry);
		flush_tlb_page_nohash(vma, address);
	}
	return changed;
}

#ifdef CONFIG_DEBUG_VM
void assert_pte_locked(struct mm_struct *mm, unsigned long addr)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;

	if (mm == &init_mm)
		return;
	pgd = mm->pgd + pgd_index(addr);
	BUG_ON(pgd_none(*pgd));
	pud = pud_offset(pgd, addr);
	BUG_ON(pud_none(*pud));
	pmd = pmd_offset(pud, addr);
	BUG_ON(!pmd_present(*pmd));
	assert_spin_locked(pte_lockptr(mm, pmd));
}
#endif /* CONFIG_DEBUG_VM */

