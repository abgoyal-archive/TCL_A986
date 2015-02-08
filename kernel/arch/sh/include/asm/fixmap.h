

#ifndef _ASM_FIXMAP_H
#define _ASM_FIXMAP_H

#include <linux/kernel.h>
#include <linux/threads.h>
#include <asm/page.h>
#ifdef CONFIG_HIGHMEM
#include <asm/kmap_types.h>
#endif


enum fixed_addresses {
	/*
	 * The FIX_CMAP entries are used by kmap_coherent() to get virtual
	 * addresses which are of a known color, and so their values are
	 * important. __fix_to_virt(FIX_CMAP_END - n) must give an address
	 * which is the same color as a page (n<<PAGE_SHIFT).
	 */
#define FIX_N_COLOURS 8
	FIX_CMAP_BEGIN,
	FIX_CMAP_END = FIX_CMAP_BEGIN + (FIX_N_COLOURS * NR_CPUS) - 1,

#ifdef CONFIG_HIGHMEM
	FIX_KMAP_BEGIN,	/* reserved pte's for temporary kernel mappings */
	FIX_KMAP_END = FIX_KMAP_BEGIN+(KM_TYPE_NR*NR_CPUS)-1,
#endif

#ifdef CONFIG_IOREMAP_FIXED
	/*
	 * FIX_IOREMAP entries are useful for mapping physical address
	 * space before ioremap() is useable, e.g. really early in boot
	 * before kmalloc() is working.
	 */
#define FIX_N_IOREMAPS	32
	FIX_IOREMAP_BEGIN,
	FIX_IOREMAP_END = FIX_IOREMAP_BEGIN + FIX_N_IOREMAPS,
#endif

	__end_of_fixed_addresses
};

extern void __set_fixmap(enum fixed_addresses idx,
			 unsigned long phys, pgprot_t flags);
extern void __clear_fixmap(enum fixed_addresses idx, pgprot_t flags);

#define set_fixmap(idx, phys) \
		__set_fixmap(idx, phys, PAGE_KERNEL)
#define set_fixmap_nocache(idx, phys) \
		__set_fixmap(idx, phys, PAGE_KERNEL_NOCACHE)
#ifdef CONFIG_SUPERH32
#define FIXADDR_TOP	(P4SEG - PAGE_SIZE)
#else
#define FIXADDR_TOP	(0xff000000 - PAGE_SIZE)
#endif
#define FIXADDR_SIZE	(__end_of_fixed_addresses << PAGE_SHIFT)
#define FIXADDR_START	(FIXADDR_TOP - FIXADDR_SIZE)

#define __fix_to_virt(x)	(FIXADDR_TOP - ((x) << PAGE_SHIFT))
#define __virt_to_fix(x)	((FIXADDR_TOP - ((x)&PAGE_MASK)) >> PAGE_SHIFT)

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
