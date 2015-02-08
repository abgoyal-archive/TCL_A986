


#ifndef LINUX_RAID_RAID6X86_H
#define LINUX_RAID_RAID6X86_H

#if (defined(__i386__) || defined(__x86_64__)) && !defined(__arch_um__)

#ifdef __KERNEL__ /* Real code */

#include <asm/i387.h>

#else /* Dummy code for user space testing */

static inline void kernel_fpu_begin(void)
{
}

static inline void kernel_fpu_end(void)
{
}

#define X86_FEATURE_MMX		(0*32+23) /* Multimedia Extensions */
#define X86_FEATURE_FXSR	(0*32+24) /* FXSAVE and FXRSTOR instructions
					   * (fast save and restore) */
#define X86_FEATURE_XMM		(0*32+25) /* Streaming SIMD Extensions */
#define X86_FEATURE_XMM2	(0*32+26) /* Streaming SIMD Extensions-2 */
#define X86_FEATURE_MMXEXT	(1*32+22) /* AMD MMX extensions */

/* Should work well enough on modern CPUs for testing */
static inline int boot_cpu_has(int flag)
{
	u32 eax = (flag >> 5) ? 0x80000001 : 1;
	u32 edx;

	asm volatile("cpuid"
		     : "+a" (eax), "=d" (edx)
		     : : "ecx", "ebx");

	return (edx >> (flag & 31)) & 1;
}

#endif /* ndef __KERNEL__ */

#endif
#endif
