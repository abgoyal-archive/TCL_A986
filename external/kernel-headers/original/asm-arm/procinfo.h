#ifndef __ASM_PROCINFO_H
#define __ASM_PROCINFO_H

#ifndef __ASSEMBLY__

struct cpu_tlb_fns;
struct cpu_user_fns;
struct cpu_cache_fns;
struct processor;

struct proc_info_list {
	unsigned int		cpu_val;
	unsigned int		cpu_mask;
	unsigned long		__cpu_mm_mmu_flags;	/* used by head.S */
	unsigned long		__cpu_io_mmu_flags;	/* used by head.S */
	unsigned long		__cpu_flush;		/* used by head.S */
	const char		*arch_name;
	const char		*elf_name;
	unsigned int		elf_hwcap;
	const char		*cpu_name;
	struct processor	*proc;
	struct cpu_tlb_fns	*tlb;
	struct cpu_user_fns	*user;
	struct cpu_cache_fns	*cache;
};

extern unsigned int elf_hwcap;

#endif	/* __ASSEMBLY__ */

#define HWCAP_SWP	1
#define HWCAP_HALF	2
#define HWCAP_THUMB	4
#define HWCAP_26BIT	8	/* Play it safe */
#define HWCAP_FAST_MULT	16
#define HWCAP_FPA	32
#define HWCAP_VFP	64
#define HWCAP_EDSP	128
#define HWCAP_JAVA	256
#define HWCAP_IWMMXT	512

#endif
