
#ifndef _ASM_MMU_CONTEXT_H
#define _ASM_MMU_CONTEXT_H

#include <asm/atomic.h>
#include <asm/pgalloc.h>
#include <asm/tlbflush.h>
#include <asm-generic/mm_hooks.h>

#define MMU_CONTEXT_TLBPID_MASK		0x000000ffUL
#define MMU_CONTEXT_VERSION_MASK	0xffffff00UL
#define MMU_CONTEXT_FIRST_VERSION	0x00000100UL
#define MMU_NO_CONTEXT			0x00000000UL

extern unsigned long mmu_context_cache[NR_CPUS];
#define mm_context(mm)	(mm->context.tlbpid[smp_processor_id()])

#define enter_lazy_tlb(mm, tsk)	do {} while (0)

#ifdef CONFIG_SMP
#define cpu_ran_vm(cpu, mm) \
	cpumask_set_cpu((cpu), mm_cpumask(mm))
#define cpu_maybe_ran_vm(cpu, mm) \
	cpumask_test_and_set_cpu((cpu), mm_cpumask(mm))
#else
#define cpu_ran_vm(cpu, mm)		do {} while (0)
#define cpu_maybe_ran_vm(cpu, mm)	true
#endif /* CONFIG_SMP */

static inline unsigned long allocate_mmu_context(struct mm_struct *mm)
{
	unsigned long *pmc = &mmu_context_cache[smp_processor_id()];
	unsigned long mc = ++(*pmc);

	if (!(mc & MMU_CONTEXT_TLBPID_MASK)) {
		/* we exhausted the TLB PIDs of this version on this CPU, so we
		 * flush this CPU's TLB in its entirety and start new cycle */
		flush_tlb_all();

		/* fix the TLB version if needed (we avoid version #0 so as to
		 * distingush MMU_NO_CONTEXT) */
		if (!mc)
			*pmc = mc = MMU_CONTEXT_FIRST_VERSION;
	}
	mm_context(mm) = mc;
	return mc;
}

static inline unsigned long get_mmu_context(struct mm_struct *mm)
{
	unsigned long mc = MMU_NO_CONTEXT, cache;

	if (mm) {
		cache = mmu_context_cache[smp_processor_id()];
		mc = mm_context(mm);

		/* if we have an old version of the context, replace it */
		if ((mc ^ cache) & MMU_CONTEXT_VERSION_MASK)
			mc = allocate_mmu_context(mm);
	}
	return mc;
}

static inline int init_new_context(struct task_struct *tsk,
				   struct mm_struct *mm)
{
	int num_cpus = NR_CPUS, i;

	for (i = 0; i < num_cpus; i++)
		mm->context.tlbpid[i] = MMU_NO_CONTEXT;
	return 0;
}

#define destroy_context(mm)	do { } while (0)

static inline void activate_context(struct mm_struct *mm, int cpu)
{
	PIDR = get_mmu_context(mm) & MMU_CONTEXT_TLBPID_MASK;
}

static inline void switch_mm(struct mm_struct *prev, struct mm_struct *next,
			     struct task_struct *tsk)
{
	int cpu = smp_processor_id();

	if (prev != next) {
		cpu_ran_vm(cpu, next);
		activate_context(next, cpu);
		PTBR = (unsigned long) next->pgd;
	} else if (!cpu_maybe_ran_vm(cpu, next)) {
		activate_context(next, cpu);
	}
}

#define deactivate_mm(tsk, mm)	do {} while (0)
#define activate_mm(prev, next)	switch_mm((prev), (next), NULL)

#endif /* _ASM_MMU_CONTEXT_H */
