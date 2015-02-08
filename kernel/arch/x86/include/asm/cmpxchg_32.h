
#ifndef _ASM_X86_CMPXCHG_32_H
#define _ASM_X86_CMPXCHG_32_H

#include <linux/bitops.h> /* for LOCK_PREFIX */


extern void __xchg_wrong_size(void);


struct __xchg_dummy {
	unsigned long a[100];
};
#define __xg(x) ((struct __xchg_dummy *)(x))

#define __xchg(x, ptr, size)						\
({									\
	__typeof(*(ptr)) __x = (x);					\
	switch (size) {							\
	case 1:								\
		asm volatile("xchgb %b0,%1"				\
			     : "=q" (__x), "+m" (*__xg(ptr))		\
			     : "0" (__x)				\
			     : "memory");				\
		break;							\
	case 2:								\
		asm volatile("xchgw %w0,%1"				\
			     : "=r" (__x), "+m" (*__xg(ptr))		\
			     : "0" (__x)				\
			     : "memory");				\
		break;							\
	case 4:								\
		asm volatile("xchgl %0,%1"				\
			     : "=r" (__x), "+m" (*__xg(ptr))		\
			     : "0" (__x)				\
			     : "memory");				\
		break;							\
	default:							\
		__xchg_wrong_size();					\
	}								\
	__x;								\
})

#define xchg(ptr, v)							\
	__xchg((v), (ptr), sizeof(*ptr))

static inline void set_64bit(volatile u64 *ptr, u64 value)
{
	u32 low  = value;
	u32 high = value >> 32;
	u64 prev = *ptr;

	asm volatile("\n1:\t"
		     LOCK_PREFIX "cmpxchg8b %0\n\t"
		     "jnz 1b"
		     : "=m" (*ptr), "+A" (prev)
		     : "b" (low), "c" (high)
		     : "memory");
}

extern void __cmpxchg_wrong_size(void);

#define __raw_cmpxchg(ptr, old, new, size, lock)			\
({									\
	__typeof__(*(ptr)) __ret;					\
	__typeof__(*(ptr)) __old = (old);				\
	__typeof__(*(ptr)) __new = (new);				\
	switch (size) {							\
	case 1:								\
		asm volatile(lock "cmpxchgb %b2,%1"			\
			     : "=a" (__ret), "+m" (*__xg(ptr))		\
			     : "q" (__new), "0" (__old)			\
			     : "memory");				\
		break;							\
	case 2:								\
		asm volatile(lock "cmpxchgw %w2,%1"			\
			     : "=a" (__ret), "+m" (*__xg(ptr))		\
			     : "r" (__new), "0" (__old)			\
			     : "memory");				\
		break;							\
	case 4:								\
		asm volatile(lock "cmpxchgl %2,%1"			\
			     : "=a" (__ret), "+m" (*__xg(ptr))		\
			     : "r" (__new), "0" (__old)			\
			     : "memory");				\
		break;							\
	default:							\
		__cmpxchg_wrong_size();					\
	}								\
	__ret;								\
})

#define __cmpxchg(ptr, old, new, size)					\
	__raw_cmpxchg((ptr), (old), (new), (size), LOCK_PREFIX)

#define __sync_cmpxchg(ptr, old, new, size)				\
	__raw_cmpxchg((ptr), (old), (new), (size), "lock; ")

#define __cmpxchg_local(ptr, old, new, size)				\
	__raw_cmpxchg((ptr), (old), (new), (size), "")

#ifdef CONFIG_X86_CMPXCHG
#define __HAVE_ARCH_CMPXCHG 1

#define cmpxchg(ptr, old, new)						\
	__cmpxchg((ptr), (old), (new), sizeof(*ptr))

#define sync_cmpxchg(ptr, old, new)					\
	__sync_cmpxchg((ptr), (old), (new), sizeof(*ptr))

#define cmpxchg_local(ptr, old, new)					\
	__cmpxchg_local((ptr), (old), (new), sizeof(*ptr))
#endif

#ifdef CONFIG_X86_CMPXCHG64
#define cmpxchg64(ptr, o, n)						\
	((__typeof__(*(ptr)))__cmpxchg64((ptr), (unsigned long long)(o), \
					 (unsigned long long)(n)))
#define cmpxchg64_local(ptr, o, n)					\
	((__typeof__(*(ptr)))__cmpxchg64_local((ptr), (unsigned long long)(o), \
					       (unsigned long long)(n)))
#endif

static inline unsigned long long __cmpxchg64(volatile void *ptr,
					     unsigned long long old,
					     unsigned long long new)
{
	unsigned long long prev;
	asm volatile(LOCK_PREFIX "cmpxchg8b %1"
		     : "=A" (prev),
		       "+m" (*__xg(ptr))
		     : "b" ((unsigned long)new),
		       "c" ((unsigned long)(new >> 32)),
		       "0" (old)
		     : "memory");
	return prev;
}

static inline unsigned long long __cmpxchg64_local(volatile void *ptr,
						   unsigned long long old,
						   unsigned long long new)
{
	unsigned long long prev;
	asm volatile("cmpxchg8b %1"
		     : "=A" (prev),
		       "+m" (*__xg(ptr))
		     : "b" ((unsigned long)new),
		       "c" ((unsigned long)(new >> 32)),
		       "0" (old)
		     : "memory");
	return prev;
}

#ifndef CONFIG_X86_CMPXCHG

extern unsigned long cmpxchg_386_u8(volatile void *, u8, u8);
extern unsigned long cmpxchg_386_u16(volatile void *, u16, u16);
extern unsigned long cmpxchg_386_u32(volatile void *, u32, u32);

static inline unsigned long cmpxchg_386(volatile void *ptr, unsigned long old,
					unsigned long new, int size)
{
	switch (size) {
	case 1:
		return cmpxchg_386_u8(ptr, old, new);
	case 2:
		return cmpxchg_386_u16(ptr, old, new);
	case 4:
		return cmpxchg_386_u32(ptr, old, new);
	}
	return old;
}

#define cmpxchg(ptr, o, n)						\
({									\
	__typeof__(*(ptr)) __ret;					\
	if (likely(boot_cpu_data.x86 > 3))				\
		__ret = (__typeof__(*(ptr)))__cmpxchg((ptr),		\
				(unsigned long)(o), (unsigned long)(n),	\
				sizeof(*(ptr)));			\
	else								\
		__ret = (__typeof__(*(ptr)))cmpxchg_386((ptr),		\
				(unsigned long)(o), (unsigned long)(n),	\
				sizeof(*(ptr)));			\
	__ret;								\
})
#define cmpxchg_local(ptr, o, n)					\
({									\
	__typeof__(*(ptr)) __ret;					\
	if (likely(boot_cpu_data.x86 > 3))				\
		__ret = (__typeof__(*(ptr)))__cmpxchg_local((ptr),	\
				(unsigned long)(o), (unsigned long)(n),	\
				sizeof(*(ptr)));			\
	else								\
		__ret = (__typeof__(*(ptr)))cmpxchg_386((ptr),		\
				(unsigned long)(o), (unsigned long)(n),	\
				sizeof(*(ptr)));			\
	__ret;								\
})
#endif

#ifndef CONFIG_X86_CMPXCHG64

extern unsigned long long cmpxchg_486_u64(volatile void *, u64, u64);

#define cmpxchg64(ptr, o, n)					\
({								\
	__typeof__(*(ptr)) __ret;				\
	__typeof__(*(ptr)) __old = (o);				\
	__typeof__(*(ptr)) __new = (n);				\
	alternative_io(LOCK_PREFIX_HERE				\
			"call cmpxchg8b_emu",			\
			"lock; cmpxchg8b (%%esi)" ,		\
		       X86_FEATURE_CX8,				\
		       "=A" (__ret),				\
		       "S" ((ptr)), "0" (__old),		\
		       "b" ((unsigned int)__new),		\
		       "c" ((unsigned int)(__new>>32))		\
		       : "memory");				\
	__ret; })



#define cmpxchg64_local(ptr, o, n)					\
({									\
	__typeof__(*(ptr)) __ret;					\
	if (likely(boot_cpu_data.x86 > 4))				\
		__ret = (__typeof__(*(ptr)))__cmpxchg64_local((ptr),	\
				(unsigned long long)(o),		\
				(unsigned long long)(n));		\
	else								\
		__ret = (__typeof__(*(ptr)))cmpxchg_486_u64((ptr),	\
				(unsigned long long)(o),		\
				(unsigned long long)(n));		\
	__ret;								\
})

#endif

#endif /* _ASM_X86_CMPXCHG_32_H */
