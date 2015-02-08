

#include <linux/sched.h>
#include <asm/suspend.h>
#include <asm/system.h>
#include <asm/current.h>
#include <asm/mmu_context.h>

void save_processor_state(void)
{
	/*
	 * flush out all the special registers so we don't need
	 * to save them in the snapshot
	 */
	flush_fp_to_thread(current);
	flush_altivec_to_thread(current);
	flush_spe_to_thread(current);

#ifdef CONFIG_PPC64
	hard_irq_disable();
#endif

}

void restore_processor_state(void)
{
#ifdef CONFIG_PPC32
	switch_mmu_context(NULL, current->active_mm);
#endif
}
