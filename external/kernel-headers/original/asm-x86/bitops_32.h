#ifndef _I386_BITOPS_H
#define _I386_BITOPS_H


#ifndef _LINUX_BITOPS_H
#error only <linux/bitops.h> can be included directly
#endif

#include <linux/compiler.h>
#include <asm/alternative.h>


#define ADDR (*(volatile long *) addr)

static inline void set_bit(int nr, volatile unsigned long * addr)
{
	__asm__ __volatile__( LOCK_PREFIX
		"btsl %1,%0"
		:"+m" (ADDR)
		:"Ir" (nr));
}

static inline void __set_bit(int nr, volatile unsigned long * addr)
{
	__asm__(
		"btsl %1,%0"
		:"+m" (ADDR)
		:"Ir" (nr));
}

static inline void clear_bit(int nr, volatile unsigned long * addr)
{
	__asm__ __volatile__( LOCK_PREFIX
		"btrl %1,%0"
		:"+m" (ADDR)
		:"Ir" (nr));
}

static inline void clear_bit_unlock(unsigned long nr, volatile unsigned long *addr)
{
	barrier();
	clear_bit(nr, addr);
}

static inline void __clear_bit(int nr, volatile unsigned long * addr)
{
	__asm__ __volatile__(
		"btrl %1,%0"
		:"+m" (ADDR)
		:"Ir" (nr));
}

static inline void __clear_bit_unlock(unsigned long nr, volatile unsigned long *addr)
{
	barrier();
	__clear_bit(nr, addr);
}

#define smp_mb__before_clear_bit()	barrier()
#define smp_mb__after_clear_bit()	barrier()

static inline void __change_bit(int nr, volatile unsigned long * addr)
{
	__asm__ __volatile__(
		"btcl %1,%0"
		:"+m" (ADDR)
		:"Ir" (nr));
}

static inline void change_bit(int nr, volatile unsigned long * addr)
{
	__asm__ __volatile__( LOCK_PREFIX
		"btcl %1,%0"
		:"+m" (ADDR)
		:"Ir" (nr));
}

static inline int test_and_set_bit(int nr, volatile unsigned long * addr)
{
	int oldbit;

	__asm__ __volatile__( LOCK_PREFIX
		"btsl %2,%1\n\tsbbl %0,%0"
		:"=r" (oldbit),"+m" (ADDR)
		:"Ir" (nr) : "memory");
	return oldbit;
}

static inline int test_and_set_bit_lock(int nr, volatile unsigned long *addr)
{
	return test_and_set_bit(nr, addr);
}

static inline int __test_and_set_bit(int nr, volatile unsigned long * addr)
{
	int oldbit;

	__asm__(
		"btsl %2,%1\n\tsbbl %0,%0"
		:"=r" (oldbit),"+m" (ADDR)
		:"Ir" (nr));
	return oldbit;
}

static inline int test_and_clear_bit(int nr, volatile unsigned long * addr)
{
	int oldbit;

	__asm__ __volatile__( LOCK_PREFIX
		"btrl %2,%1\n\tsbbl %0,%0"
		:"=r" (oldbit),"+m" (ADDR)
		:"Ir" (nr) : "memory");
	return oldbit;
}

static inline int __test_and_clear_bit(int nr, volatile unsigned long *addr)
{
	int oldbit;

	__asm__(
		"btrl %2,%1\n\tsbbl %0,%0"
		:"=r" (oldbit),"+m" (ADDR)
		:"Ir" (nr));
	return oldbit;
}

/* WARNING: non atomic and it can be reordered! */
static inline int __test_and_change_bit(int nr, volatile unsigned long *addr)
{
	int oldbit;

	__asm__ __volatile__(
		"btcl %2,%1\n\tsbbl %0,%0"
		:"=r" (oldbit),"+m" (ADDR)
		:"Ir" (nr) : "memory");
	return oldbit;
}

static inline int test_and_change_bit(int nr, volatile unsigned long* addr)
{
	int oldbit;

	__asm__ __volatile__( LOCK_PREFIX
		"btcl %2,%1\n\tsbbl %0,%0"
		:"=r" (oldbit),"+m" (ADDR)
		:"Ir" (nr) : "memory");
	return oldbit;
}

#if 0 /* Fool kernel-doc since it doesn't do macros yet */
static int test_bit(int nr, const volatile void * addr);
#endif

static __always_inline int constant_test_bit(int nr, const volatile unsigned long *addr)
{
	return ((1UL << (nr & 31)) & (addr[nr >> 5])) != 0;
}

static inline int variable_test_bit(int nr, const volatile unsigned long * addr)
{
	int oldbit;

	__asm__ __volatile__(
		"btl %2,%1\n\tsbbl %0,%0"
		:"=r" (oldbit)
		:"m" (ADDR),"Ir" (nr));
	return oldbit;
}

#define test_bit(nr,addr) \
(__builtin_constant_p(nr) ? \
 constant_test_bit((nr),(addr)) : \
 variable_test_bit((nr),(addr)))

#undef ADDR

static inline int find_first_zero_bit(const unsigned long *addr, unsigned size)
{
	int d0, d1, d2;
	int res;

	if (!size)
		return 0;
	/* This looks at memory. Mark it volatile to tell gcc not to move it around */
	__asm__ __volatile__(
		"movl $-1,%%eax\n\t"
		"xorl %%edx,%%edx\n\t"
		"repe; scasl\n\t"
		"je 1f\n\t"
		"xorl -4(%%edi),%%eax\n\t"
		"subl $4,%%edi\n\t"
		"bsfl %%eax,%%edx\n"
		"1:\tsubl %%ebx,%%edi\n\t"
		"shll $3,%%edi\n\t"
		"addl %%edi,%%edx"
		:"=d" (res), "=&c" (d0), "=&D" (d1), "=&a" (d2)
		:"1" ((size + 31) >> 5), "2" (addr), "b" (addr) : "memory");
	return res;
}

int find_next_zero_bit(const unsigned long *addr, int size, int offset);

static inline unsigned long __ffs(unsigned long word)
{
	__asm__("bsfl %1,%0"
		:"=r" (word)
		:"rm" (word));
	return word;
}

static inline unsigned find_first_bit(const unsigned long *addr, unsigned size)
{
	unsigned x = 0;

	while (x < size) {
		unsigned long val = *addr++;
		if (val)
			return __ffs(val) + x;
		x += (sizeof(*addr)<<3);
	}
	return x;
}

int find_next_bit(const unsigned long *addr, int size, int offset);

static inline unsigned long ffz(unsigned long word)
{
	__asm__("bsfl %1,%0"
		:"=r" (word)
		:"r" (~word));
	return word;
}

#ifdef __KERNEL__

#include <asm-generic/bitops/sched.h>

static inline int ffs(int x)
{
	int r;

	__asm__("bsfl %1,%0\n\t"
		"jnz 1f\n\t"
		"movl $-1,%0\n"
		"1:" : "=r" (r) : "rm" (x));
	return r+1;
}

static inline int fls(int x)
{
	int r;

	__asm__("bsrl %1,%0\n\t"
		"jnz 1f\n\t"
		"movl $-1,%0\n"
		"1:" : "=r" (r) : "rm" (x));
	return r+1;
}

#include <asm-generic/bitops/hweight.h>

#endif /* __KERNEL__ */

#include <asm-generic/bitops/fls64.h>

#ifdef __KERNEL__

#include <asm-generic/bitops/ext2-non-atomic.h>

#define ext2_set_bit_atomic(lock,nr,addr) \
        test_and_set_bit((nr),(unsigned long*)addr)
#define ext2_clear_bit_atomic(lock,nr, addr) \
	        test_and_clear_bit((nr),(unsigned long*)addr)

#include <asm-generic/bitops/minix.h>

#endif /* __KERNEL__ */

#endif /* _I386_BITOPS_H */
