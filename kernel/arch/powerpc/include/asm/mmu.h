
#ifndef _ASM_POWERPC_MMU_H_
#define _ASM_POWERPC_MMU_H_
#ifdef __KERNEL__

#include <asm/asm-compat.h>
#include <asm/feature-fixups.h>


#define MMU_FTR_HPTE_TABLE		ASM_CONST(0x00000001)
#define MMU_FTR_TYPE_8xx		ASM_CONST(0x00000002)
#define MMU_FTR_TYPE_40x		ASM_CONST(0x00000004)
#define MMU_FTR_TYPE_44x		ASM_CONST(0x00000008)
#define MMU_FTR_TYPE_FSL_E		ASM_CONST(0x00000010)
#define MMU_FTR_TYPE_3E			ASM_CONST(0x00000020)
#define MMU_FTR_TYPE_47x		ASM_CONST(0x00000040)


/* Enable use of high BAT registers */
#define MMU_FTR_USE_HIGH_BATS		ASM_CONST(0x00010000)

#define MMU_FTR_BIG_PHYS		ASM_CONST(0x00020000)

#define MMU_FTR_USE_TLBIVAX_BCAST	ASM_CONST(0x00040000)

#define MMU_FTR_USE_TLBILX		ASM_CONST(0x00080000)

#define MMU_FTR_LOCK_BCAST_INVAL	ASM_CONST(0x00100000)

#define MMU_FTR_NEED_DTLB_SW_LRU	ASM_CONST(0x00200000)

#define MMU_FTR_TLBIE_206		ASM_CONST(0x00400000)

#define MMU_FTR_USE_TLBRSRV		ASM_CONST(0x00800000)

#define MMU_FTR_USE_PAIRED_MAS		ASM_CONST(0x01000000)

#ifndef __ASSEMBLY__
#include <asm/cputable.h>

static inline int mmu_has_feature(unsigned long feature)
{
	return (cur_cpu_spec->mmu_features & feature);
}

extern unsigned int __start___mmu_ftr_fixup, __stop___mmu_ftr_fixup;

/* MMU initialization (64-bit only fo now) */
extern void early_init_mmu(void);
extern void early_init_mmu_secondary(void);

#endif /* !__ASSEMBLY__ */


#define MMU_PAGE_4K	0
#define MMU_PAGE_16K	1
#define MMU_PAGE_64K	2
#define MMU_PAGE_64K_AP	3	/* "Admixed pages" (hash64 only) */
#define MMU_PAGE_256K	4
#define MMU_PAGE_1M	5
#define MMU_PAGE_8M	6
#define MMU_PAGE_16M	7
#define MMU_PAGE_256M	8
#define MMU_PAGE_1G	9
#define MMU_PAGE_16G	10
#define MMU_PAGE_64G	11
#define MMU_PAGE_COUNT	12


#if defined(CONFIG_PPC_STD_MMU_64)
/* 64-bit classic hash table MMU */
#  include <asm/mmu-hash64.h>
#elif defined(CONFIG_PPC_STD_MMU_32)
/* 32-bit classic hash table MMU */
#  include <asm/mmu-hash32.h>
#elif defined(CONFIG_40x)
/* 40x-style software loaded TLB */
#  include <asm/mmu-40x.h>
#elif defined(CONFIG_44x)
/* 44x-style software loaded TLB */
#  include <asm/mmu-44x.h>
#elif defined(CONFIG_PPC_BOOK3E_MMU)
/* Freescale Book-E software loaded TLB or Book-3e (ISA 2.06+) MMU */
#  include <asm/mmu-book3e.h>
#elif defined (CONFIG_PPC_8xx)
/* Motorola/Freescale 8xx software loaded TLB */
#  include <asm/mmu-8xx.h>
#endif


#endif /* __KERNEL__ */
#endif /* _ASM_POWERPC_MMU_H_ */
