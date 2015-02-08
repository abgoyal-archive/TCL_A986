#ifndef __ASM_ARM_BYTEORDER_H
#define __ASM_ARM_BYTEORDER_H

#include <linux/compiler.h>
#include <asm/types.h>

static inline __attribute_const__ __u32 ___arch__swab32(__u32 x)
{
	__u32 t;

#ifndef __thumb__
	if (!__builtin_constant_p(x)) {
		/*
		 * The compiler needs a bit of a hint here to always do the
		 * right thing and not screw it up to different degrees
		 * depending on the gcc version.
		 */
		asm ("eor\t%0, %1, %1, ror #16" : "=r" (t) : "r" (x));
	} else
#endif
		t = x ^ ((x << 16) | (x >> 16)); /* eor r1,r0,r0,ror #16 */

	x = (x << 24) | (x >> 8);		/* mov r0,r0,ror #8      */
	t &= ~0x00FF0000;			/* bic r1,r1,#0x00FF0000 */
	x ^= (t >> 8);				/* eor r0,r0,r1,lsr #8   */

	return x;
}

#define __arch__swab32(x) ___arch__swab32(x)

#if !defined(__STRICT_ANSI__) || defined(__KERNEL__)
#  define __BYTEORDER_HAS_U64__
#  define __SWAB_64_THRU_32__
#endif

#ifdef __ARMEB__
#include <linux/byteorder/big_endian.h>
#else
#include <linux/byteorder/little_endian.h>
#endif

#endif

