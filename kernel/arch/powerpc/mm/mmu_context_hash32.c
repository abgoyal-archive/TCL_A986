

#include <linux/mm.h>
#include <linux/init.h>

#include <asm/mmu_context.h>
#include <asm/tlbflush.h>

#define NO_CONTEXT      	((unsigned long) -1)
#define LAST_CONTEXT    	32767
#define FIRST_CONTEXT    	1


static unsigned long next_mmu_context;
static unsigned long context_map[LAST_CONTEXT / BITS_PER_LONG + 1];

unsigned long __init_new_context(void)
{
	unsigned long ctx = next_mmu_context;

	while (test_and_set_bit(ctx, context_map)) {
		ctx = find_next_zero_bit(context_map, LAST_CONTEXT+1, ctx);
		if (ctx > LAST_CONTEXT)
			ctx = 0;
	}
	next_mmu_context = (ctx + 1) & LAST_CONTEXT;

	return ctx;
}
EXPORT_SYMBOL_GPL(__init_new_context);

int init_new_context(struct task_struct *t, struct mm_struct *mm)
{
	mm->context.id = __init_new_context();

	return 0;
}

void __destroy_context(unsigned long ctx)
{
	clear_bit(ctx, context_map);
}
EXPORT_SYMBOL_GPL(__destroy_context);

void destroy_context(struct mm_struct *mm)
{
	preempt_disable();
	if (mm->context.id != NO_CONTEXT) {
		__destroy_context(mm->context.id);
		mm->context.id = NO_CONTEXT;
	}
	preempt_enable();
}

void __init mmu_context_init(void)
{
	/* Reserve context 0 for kernel use */
	context_map[0] = (1 << FIRST_CONTEXT) - 1;
	next_mmu_context = FIRST_CONTEXT;
}
