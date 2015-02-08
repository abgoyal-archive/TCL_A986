

#include <linux/mm.h>
#include <linux/init.h>

#include <asm/tlbflush.h>
#include <asm/mmu_context.h>

mm_context_t next_mmu_context;
unsigned long context_map[LAST_CONTEXT / BITS_PER_LONG + 1];
atomic_t nr_free_contexts;
struct mm_struct *context_mm[LAST_CONTEXT+1];

void __init mmu_context_init(void)
{
	/*
	 * The use of context zero is reserved for the kernel.
	 * This code assumes FIRST_CONTEXT < 32.
	 */
	context_map[0] = (1 << FIRST_CONTEXT) - 1;
	next_mmu_context = FIRST_CONTEXT;
	atomic_set(&nr_free_contexts, LAST_CONTEXT - FIRST_CONTEXT + 1);
}

void steal_context(void)
{
	struct mm_struct *mm;

	/* free up context `next_mmu_context' */
	/* if we shouldn't free context 0, don't... */
	if (next_mmu_context < FIRST_CONTEXT)
		next_mmu_context = FIRST_CONTEXT;
	mm = context_mm[next_mmu_context];
	flush_tlb_mm(mm);
	destroy_context(mm);
}
