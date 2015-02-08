

#include <linux/init.h>
#include <linux/kernel.h>
#include <asm/tlb.h>

#define D(x)


struct mm_struct *page_id_map[NUM_PAGEID];
static int map_replace_ptr = 1;  /* which page_id_map entry to replace next */

/* the following functions are similar to those used in the PPC port */

static inline void
alloc_context(struct mm_struct *mm)
{
	struct mm_struct *old_mm;

	D(printk("tlb: alloc context %d (%p)\n", map_replace_ptr, mm));

	/* did we replace an mm ? */

	old_mm = page_id_map[map_replace_ptr];

	if(old_mm) {
		/* throw out any TLB entries belonging to the mm we replace
		 * in the map
		 */
		flush_tlb_mm(old_mm);

		old_mm->context.page_id = NO_CONTEXT;
	}

	/* insert it into the page_id_map */

	mm->context.page_id = map_replace_ptr;
	page_id_map[map_replace_ptr] = mm;

	map_replace_ptr++;

	if(map_replace_ptr == INVALID_PAGEID)
		map_replace_ptr = 0;         /* wrap around */	
}


void
get_mmu_context(struct mm_struct *mm)
{
	if(mm->context.page_id == NO_CONTEXT)
		alloc_context(mm);
}


void
destroy_context(struct mm_struct *mm)
{
	if(mm->context.page_id != NO_CONTEXT) {
		D(printk("destroy_context %d (%p)\n", mm->context.page_id, mm));
		flush_tlb_mm(mm);  /* TODO this might be redundant ? */
		page_id_map[mm->context.page_id] = NULL;
	}
}

/* called once during VM initialization, from init.c */

void __init
tlb_init(void)
{
	int i;

	/* clear the page_id map */

	for (i = 1; i < ARRAY_SIZE(page_id_map); i++)
		page_id_map[i] = NULL;
	
	/* invalidate the entire TLB */

	flush_tlb_all();

	/* the init_mm has context 0 from the boot */

	page_id_map[0] = &init_mm;
}
