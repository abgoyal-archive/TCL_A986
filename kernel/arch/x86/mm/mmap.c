

#include <linux/personality.h>
#include <linux/mm.h>
#include <linux/random.h>
#include <linux/limits.h>
#include <linux/sched.h>
#include <asm/elf.h>

static unsigned int stack_maxrandom_size(void)
{
	unsigned int max = 0;
	if ((current->flags & PF_RANDOMIZE) &&
		!(current->personality & ADDR_NO_RANDOMIZE)) {
		max = ((-1U) & STACK_RND_MASK) << PAGE_SHIFT;
	}

	return max;
}


#define MIN_GAP (128*1024*1024UL + stack_maxrandom_size())
#define MAX_GAP (TASK_SIZE/6*5)

static int mmap_is_ia32(void)
{
#ifdef CONFIG_X86_32
	return 1;
#endif
#ifdef CONFIG_IA32_EMULATION
	if (test_thread_flag(TIF_IA32))
		return 1;
#endif
	return 0;
}

static int mmap_is_legacy(void)
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

	/*
	*  8 bits of randomness in 32bit mmaps, 20 address space bits
	* 28 bits of randomness in 64bit mmaps, 40 address space bits
	*/
	if (current->flags & PF_RANDOMIZE) {
		if (mmap_is_ia32())
			rnd = (long)get_random_int() % (1<<8);
		else
			rnd = (long)(get_random_int() % (1<<28));
	}
	return rnd << PAGE_SHIFT;
}

static unsigned long mmap_base(void)
{
	unsigned long gap = rlimit(RLIMIT_STACK);

	if (gap < MIN_GAP)
		gap = MIN_GAP;
	else if (gap > MAX_GAP)
		gap = MAX_GAP;

	return PAGE_ALIGN(TASK_SIZE - gap - mmap_rnd());
}

static unsigned long mmap_legacy_base(void)
{
	if (mmap_is_ia32())
		return TASK_UNMAPPED_BASE;
	else
		return TASK_UNMAPPED_BASE + mmap_rnd();
}

void arch_pick_mmap_layout(struct mm_struct *mm)
{
	if (mmap_is_legacy()) {
		mm->mmap_base = mmap_legacy_base();
		mm->get_unmapped_area = arch_get_unmapped_area;
		mm->unmap_area = arch_unmap_area;
	} else {
		mm->mmap_base = mmap_base();
		mm->get_unmapped_area = arch_get_unmapped_area_topdown;
		mm->unmap_area = arch_unmap_area_topdown;
	}
}
