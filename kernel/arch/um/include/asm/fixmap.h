
#ifndef __UM_FIXMAP_H
#define __UM_FIXMAP_H

#include <asm/processor.h>
#include <asm/system.h>
#include <asm/kmap_types.h>
#include <asm/archparam.h>
#include <asm/page.h>
#include <linux/threads.h>


enum fixed_addresses {
#ifdef CONFIG_HIGHMEM
	FIX_KMAP_BEGIN,	/* reserved pte's for temporary kernel mappings */
	FIX_KMAP_END = FIX_KMAP_BEGIN+(KM_TYPE_NR*NR_CPUS)-1,
#endif
	__end_of_fixed_addresses
};

extern void __set_fixmap (enum fixed_addresses idx,
			  unsigned long phys, pgprot_t flags);

#define set_fixmap(idx, phys) \
		__set_fixmap(idx, phys, PAGE_KERNEL)
#define set_fixmap_nocache(idx, phys) \
		__set_fixmap(idx, phys, PAGE_KERNEL_NOCACHE)

#define FIXADDR_TOP	(TASK_SIZE - 2 * PAGE_SIZE)
#define FIXADDR_SIZE	(__end_of_fixed_addresses << PAGE_SHIFT)
#define FIXADDR_START	(FIXADDR_TOP - FIXADDR_SIZE)

#define __fix_to_virt(x)	(FIXADDR_TOP - ((x) << PAGE_SHIFT))
#define __virt_to_fix(x)      ((FIXADDR_TOP - ((x)&PAGE_MASK)) >> PAGE_SHIFT)

extern void __this_fixmap_does_not_exist(void);

static inline unsigned long fix_to_virt(const unsigned int idx)
{
	/*
	 * this branch gets completely eliminated after inlining,
	 * except when someone tries to use fixaddr indices in an
	 * illegal way. (such as mixing up address types or using
	 * out-of-range indices).
	 *
	 * If it doesn't get removed, the linker will complain
	 * loudly with a reasonably clear error message..
	 */
	if (idx >= __end_of_fixed_addresses)
		__this_fixmap_does_not_exist();

        return __fix_to_virt(idx);
}

static inline unsigned long virt_to_fix(const unsigned long vaddr)
{
      BUG_ON(vaddr >= FIXADDR_TOP || vaddr < FIXADDR_START);
      return __virt_to_fix(vaddr);
}

#endif
