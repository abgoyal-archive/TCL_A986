

#include <linux/personality.h>
#include <linux/mm.h>
#include <linux/random.h>
#include <linux/sched.h>

#define MIN_GAP32 (128*1024*1024)
#define MIN_GAP64 ((128 + 1024)*1024*1024UL)
#define MIN_GAP ((is_32bit_task()) ? MIN_GAP32 : MIN_GAP64)
#define MAX_GAP (TASK_SIZE/6*5)

static inline int mmap_is_legacy(void)
{
	if (current->personality & ADDR_COMPAT_LAYOUT)
		return 1;

	if (rlimit(RLIMIT_STACK) == RLIM_INFINITY)
		return 1;

	return sysctl_legacy_va_layout;
}

static unsigned long mmap_rnd(void)
{
	unsigned long rnd = 0;

	if (current->flags & PF_RANDOMIZE) {
		/* 8MB for 32bit, 1GB for 64bit */
		if (is_32bit_task())
			rnd = (long)(get_random_int() % (1<<(22-PAGE_SHIFT)));
		else
			rnd = (long)(get_random_int() % (1<<(29-PAGE_SHIFT)));
	}
	return (rnd << PAGE_SHIFT) * 2;
}

static inline unsigned long mmap_base(void)
{
	unsigned long gap = rlimit(RLIMIT_STACK);

	if (gap < MIN_GAP)
		gap = MIN_GAP;
	else if (gap > MAX_GAP)
		gap = MAX_GAP;

	return PAGE_ALIGN(TASK_SIZE - gap - mmap_rnd());
}

void arch_pick_mmap_layout(struct mm_struct *mm)
{
	/*
	 * Fall back to the standard layout if the personality
	 * bit is set, or if the expected stack growth is unlimited:
	 */
	if (mmap_is_legacy()) {
		mm->mmap_base = TASK_UNMAPPED_BASE;
		mm->get_unmapped_area = arch_get_unmapped_area;
		mm->unmap_area = arch_unmap_area;
	} else {
		mm->mmap_base = mmap_base();
		mm->get_unmapped_area = arch_get_unmapped_area_topdown;
		mm->unmap_area = arch_unmap_area_topdown;
	}
}
