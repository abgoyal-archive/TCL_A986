
#ifndef _ASM_SWAB_H
#define _ASM_SWAB_H

#include <linux/types.h>

#ifdef __GNUC__

static inline __attribute__((const))
__u32 __arch_swab32(__u32 x)
{
	__u32 ret;
	asm("swap %1,%0" : "=r" (ret) : "r" (x));
	return ret;
}
#define __arch_swab32 __arch_swab32

static inline __attribute__((const))
__u16 __arch_swab16(__u16 x)
{
	__u16 ret;
	asm("swaph %1,%0" : "=r" (ret) : "r" (x));
	return ret;
}
#define __arch_swab32 __arch_swab32

#if !defined(__STRICT_ANSI__) || defined(__KERNEL__)
#  define __SWAB_64_THRU_32__
#endif

#endif /* __GNUC__ */

#endif /* _ASM_SWAB_H */
