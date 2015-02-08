
#ifndef _ASM_X86_ATOMIC64_32_H
#define _ASM_X86_ATOMIC64_32_H

#include <linux/compiler.h>
#include <linux/types.h>
#include <asm/processor.h>
//#include <asm/cmpxchg.h>

/* An 64bit atomic type */

typedef struct {
	u64 __aligned(8) counter;
} atomic64_t;

#define ATOMIC64_INIT(val)	{ (val) }

#ifdef CONFIG_X86_CMPXCHG64
#define ATOMIC64_ALTERNATIVE_(f, g) "call atomic64_" #g "_cx8"
#else
#define ATOMIC64_ALTERNATIVE_(f, g) ALTERNATIVE("call atomic64_" #f "_386", "call atomic64_" #g "_cx8", X86_FEATURE_CX8)
#endif

#define ATOMIC64_ALTERNATIVE(f) ATOMIC64_ALTERNATIVE_(f, f)


static inline long long atomic64_cmpxchg(atomic64_t *v, long long o, long long n)
{
	return cmpxchg64(&v->counter, o, n);
}

static inline long long atomic64_xchg(atomic64_t *v, long long n)
{
	long long o;
	unsigned high = (unsigned)(n >> 32);
	unsigned low = (unsigned)n;
	asm volatile(ATOMIC64_ALTERNATIVE(xchg)
		     : "=A" (o), "+b" (low), "+c" (high)
		     : "S" (v)
		     : "memory"
		     );
	return o;
}

static inline void atomic64_set(atomic64_t *v, long long i)
{
	unsigned high = (unsigned)(i >> 32);
	unsigned low = (unsigned)i;
	asm volatile(ATOMIC64_ALTERNATIVE(set)
		     : "+b" (low), "+c" (high)
		     : "S" (v)
		     : "eax", "edx", "memory"
		     );
}

static inline long long atomic64_read(atomic64_t *v)
{
	long long r;
	asm volatile(ATOMIC64_ALTERNATIVE(read)
		     : "=A" (r), "+c" (v)
		     : : "memory"
		     );
	return r;
 }

static inline long long atomic64_add_return(long long i, atomic64_t *v)
{
	asm volatile(ATOMIC64_ALTERNATIVE(add_return)
		     : "+A" (i), "+c" (v)
		     : : "memory"
		     );
	return i;
}

static inline long long atomic64_sub_return(long long i, atomic64_t *v)
{
	asm volatile(ATOMIC64_ALTERNATIVE(sub_return)
		     : "+A" (i), "+c" (v)
		     : : "memory"
		     );
	return i;
}

static inline long long atomic64_inc_return(atomic64_t *v)
{
	long long a;
	asm volatile(ATOMIC64_ALTERNATIVE(inc_return)
		     : "=A" (a)
		     : "S" (v)
		     : "memory", "ecx"
		     );
	return a;
}

static inline long long atomic64_dec_return(atomic64_t *v)
{
	long long a;
	asm volatile(ATOMIC64_ALTERNATIVE(dec_return)
		     : "=A" (a)
		     : "S" (v)
		     : "memory", "ecx"
		     );
	return a;
}

static inline long long atomic64_add(long long i, atomic64_t *v)
{
	asm volatile(ATOMIC64_ALTERNATIVE_(add, add_return)
		     : "+A" (i), "+c" (v)
		     : : "memory"
		     );
	return i;
}

static inline long long atomic64_sub(long long i, atomic64_t *v)
{
	asm volatile(ATOMIC64_ALTERNATIVE_(sub, sub_return)
		     : "+A" (i), "+c" (v)
		     : : "memory"
		     );
	return i;
}

static inline int atomic64_sub_and_test(long long i, atomic64_t *v)
{
	return atomic64_sub_return(i, v) == 0;
}

static inline void atomic64_inc(atomic64_t *v)
{
	asm volatile(ATOMIC64_ALTERNATIVE_(inc, inc_return)
		     : : "S" (v)
		     : "memory", "eax", "ecx", "edx"
		     );
}

static inline void atomic64_dec(atomic64_t *v)
{
	asm volatile(ATOMIC64_ALTERNATIVE_(dec, dec_return)
		     : : "S" (v)
		     : "memory", "eax", "ecx", "edx"
		     );
}

static inline int atomic64_dec_and_test(atomic64_t *v)
{
	return atomic64_dec_return(v) == 0;
}

static inline int atomic64_inc_and_test(atomic64_t *v)
{
	return atomic64_inc_return(v) == 0;
}

static inline int atomic64_add_negative(long long i, atomic64_t *v)
{
	return atomic64_add_return(i, v) < 0;
}

static inline int atomic64_add_unless(atomic64_t *v, long long a, long long u)
{
	unsigned low = (unsigned)u;
	unsigned high = (unsigned)(u >> 32);
	asm volatile(ATOMIC64_ALTERNATIVE(add_unless) "\n\t"
		     : "+A" (a), "+c" (v), "+S" (low), "+D" (high)
		     : : "memory");
	return (int)a;
}


static inline int atomic64_inc_not_zero(atomic64_t *v)
{
	int r;
	asm volatile(ATOMIC64_ALTERNATIVE(inc_not_zero)
		     : "=a" (r)
		     : "S" (v)
		     : "ecx", "edx", "memory"
		     );
	return r;
}

static inline long long atomic64_dec_if_positive(atomic64_t *v)
{
	long long r;
	asm volatile(ATOMIC64_ALTERNATIVE(dec_if_positive)
		     : "=A" (r)
		     : "S" (v)
		     : "ecx", "memory"
		     );
	return r;
}

#undef ATOMIC64_ALTERNATIVE
#undef ATOMIC64_ALTERNATIVE_

#endif /* _ASM_X86_ATOMIC64_32_H */
