

#ifndef _XTENSA_BITOPS_H
#define _XTENSA_BITOPS_H

#ifdef __KERNEL__

#ifndef _LINUX_BITOPS_H
#error only <linux/bitops.h> can be included directly
#endif

#include <asm/processor.h>
#include <asm/byteorder.h>
#include <asm/system.h>

#ifdef CONFIG_SMP
# error SMP not supported on this architecture
#endif

#define smp_mb__before_clear_bit()	barrier()
#define smp_mb__after_clear_bit()	barrier()

#include <asm-generic/bitops/atomic.h>
#include <asm-generic/bitops/non-atomic.h>

#if XCHAL_HAVE_NSA

static inline unsigned long __cntlz (unsigned long x)
{
	int lz;
	asm ("nsau %0, %1" : "=r" (lz) : "r" (x));
	return lz;
}


static inline int ffz(unsigned long x)
{
	return 31 - __cntlz(~x & -~x);
}


static inline int __ffs(unsigned long x)
{
	return 31 - __cntlz(x & -x);
}


static inline int ffs(unsigned long x)
{
	return 32 - __cntlz(x & -x);
}


static inline int fls (unsigned int x)
{
	return 32 - __cntlz(x);
}

static inline unsigned long __fls(unsigned long word)
{
	return 31 - __cntlz(word);
}
#else

/* Use the generic implementation if we don't have the nsa/nsau instructions. */

# include <asm-generic/bitops/ffs.h>
# include <asm-generic/bitops/__ffs.h>
# include <asm-generic/bitops/ffz.h>
# include <asm-generic/bitops/fls.h>
# include <asm-generic/bitops/__fls.h>

#endif

#include <asm-generic/bitops/fls64.h>
#include <asm-generic/bitops/find.h>
#include <asm-generic/bitops/ext2-non-atomic.h>

#ifdef __XTENSA_EL__
# define ext2_set_bit_atomic(lock,nr,addr)				\
	test_and_set_bit((nr), (unsigned long*)(addr))
# define ext2_clear_bit_atomic(lock,nr,addr)				\
	test_and_clear_bit((nr), (unsigned long*)(addr))
#elif defined(__XTENSA_EB__)
# define ext2_set_bit_atomic(lock,nr,addr)				\
	test_and_set_bit((nr) ^ 0x18, (unsigned long*)(addr))
# define ext2_clear_bit_atomic(lock,nr,addr)				\
	test_and_clear_bit((nr) ^ 0x18, (unsigned long*)(addr))
#else
# error processor byte order undefined!
#endif

#include <asm-generic/bitops/hweight.h>
#include <asm-generic/bitops/lock.h>
#include <asm-generic/bitops/sched.h>
#include <asm-generic/bitops/minix.h>

#endif	/* __KERNEL__ */

#endif	/* _XTENSA_BITOPS_H */
