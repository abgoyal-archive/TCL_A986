

#ifndef _ASM_FIXMAP_H
#define _ASM_FIXMAP_H

#include <asm/page.h>
#ifdef CONFIG_HIGHMEM
#include <linux/threads.h>
#include <asm/kmap_types.h>
#endif


enum fixed_addresses {
#define FIX_N_COLOURS 8
	FIX_CMAP_BEGIN,
#ifdef CONFIG_MIPS_MT_SMTC
	FIX_CMAP_END = FIX_CMAP_BEGIN + (FIX_N_COLOURS * NR_CPUS * 2),
#else
	FIX_CMAP_END = FIX_CMAP_BEGIN + (FIX_N_COLOURS * 2),
#endif
#ifdef CONFIG_HIGHMEM
	/* reserved pte's for temporary kernel mappings */
	FIX_KMAP_BEGIN = FIX_CMAP_END + 1,
	FIX_KMAP_END = FIX_KMAP_BEGIN+(KM_TYPE_NR*NR_CPUS)-1,
#endif
	__end_of_fixed_addresses
};

#ifdef CONFIG_BCM63XX
#define FIXADDR_TOP     ((unsigned long)(long)(int)0xff000000)
#else
#if defined(CONFIG_CPU_TX39XX) || defined(CONFIG_CPU_TX49XX)
#define FIXADDR_TOP	((unsigned long)(long)(int)(0xff000000 - 0x20000))
#else
#define FIXADDR_TOP	((unsigned long)(long)(int)0xfffe0000)
#endif
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

#define kmap_get_fixmap_pte(vaddr)					\
	pte_offset_kernel(pmd_offset(pud_offset(pgd_offset_k(vaddr), (vaddr)), (vaddr)), (vaddr))

extern void fixrange_init(unsigned long start, unsigned long end,
        pgd_t *pgd_base);


#endif
