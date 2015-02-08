
#ifndef _M68K_SYSTEM_H
#define _M68K_SYSTEM_H

#include <linux/linkage.h>
#include <linux/kernel.h>
#include <asm/segment.h>
#include <asm/entry.h>

#ifdef __KERNEL__

asmlinkage void resume(void);
#define switch_to(prev,next,last) do { \
  register void *_prev __asm__ ("a0") = (prev); \
  register void *_next __asm__ ("a1") = (next); \
  register void *_last __asm__ ("d1"); \
  __asm__ __volatile__("jbsr resume" \
		       : "=a" (_prev), "=a" (_next), "=d" (_last) \
		       : "0" (_prev), "1" (_next) \
		       : "d0", "d2", "d3", "d4", "d5"); \
  (last) = _last; \
} while (0)


#define nop()		do { asm volatile ("nop"); barrier(); } while (0)
#define mb()		barrier()
#define rmb()		barrier()
#define wmb()		barrier()
#define read_barrier_depends()	((void)0)
#define set_mb(var, value)	({ (var) = (value); wmb(); })

#define smp_mb()	barrier()
#define smp_rmb()	barrier()
#define smp_wmb()	barrier()
#define smp_read_barrier_depends()	((void)0)

/* interrupt control.. */
#if 0
#define local_irq_enable() asm volatile ("andiw %0,%%sr": : "i" (ALLOWINT) : "memory")
#else
#include <linux/hardirq.h>
#define local_irq_enable() ({							\
	if (MACH_IS_Q40 || !hardirq_count())					\
		asm volatile ("andiw %0,%%sr": : "i" (ALLOWINT) : "memory");	\
})
#endif
#define local_irq_disable() asm volatile ("oriw  #0x0700,%%sr": : : "memory")
#define local_save_flags(x) asm volatile ("movew %%sr,%0":"=d" (x) : : "memory")
#define local_irq_restore(x) asm volatile ("movew %0,%%sr": :"d" (x) : "memory")

static inline int irqs_disabled(void)
{
	unsigned long flags;
	local_save_flags(flags);
	return flags & ~ALLOWINT;
}

/* For spinlocks etc */
#define local_irq_save(x)	({ local_save_flags(x); local_irq_disable(); })

#define xchg(ptr,x) ((__typeof__(*(ptr)))__xchg((unsigned long)(x),(ptr),sizeof(*(ptr))))

struct __xchg_dummy { unsigned long a[100]; };
#define __xg(x) ((volatile struct __xchg_dummy *)(x))

#ifndef CONFIG_RMW_INSNS
static inline unsigned long __xchg(unsigned long x, volatile void * ptr, int size)
{
	unsigned long flags, tmp;

	local_irq_save(flags);

	switch (size) {
	case 1:
		tmp = *(u8 *)ptr;
		*(u8 *)ptr = x;
		x = tmp;
		break;
	case 2:
		tmp = *(u16 *)ptr;
		*(u16 *)ptr = x;
		x = tmp;
		break;
	case 4:
		tmp = *(u32 *)ptr;
		*(u32 *)ptr = x;
		x = tmp;
		break;
	default:
		BUG();
	}

	local_irq_restore(flags);
	return x;
}
#else
static inline unsigned long __xchg(unsigned long x, volatile void * ptr, int size)
{
	switch (size) {
	    case 1:
		__asm__ __volatile__
			("moveb %2,%0\n\t"
			 "1:\n\t"
			 "casb %0,%1,%2\n\t"
			 "jne 1b"
			 : "=&d" (x) : "d" (x), "m" (*__xg(ptr)) : "memory");
		break;
	    case 2:
		__asm__ __volatile__
			("movew %2,%0\n\t"
			 "1:\n\t"
			 "casw %0,%1,%2\n\t"
			 "jne 1b"
			 : "=&d" (x) : "d" (x), "m" (*__xg(ptr)) : "memory");
		break;
	    case 4:
		__asm__ __volatile__
			("movel %2,%0\n\t"
			 "1:\n\t"
			 "casl %0,%1,%2\n\t"
			 "jne 1b"
			 : "=&d" (x) : "d" (x), "m" (*__xg(ptr)) : "memory");
		break;
	}
	return x;
}
#endif

#include <asm-generic/cmpxchg-local.h>

#define cmpxchg64_local(ptr, o, n) __cmpxchg64_local_generic((ptr), (o), (n))

#ifdef CONFIG_RMW_INSNS
#define __HAVE_ARCH_CMPXCHG	1

static inline unsigned long __cmpxchg(volatile void *p, unsigned long old,
				      unsigned long new, int size)
{
	switch (size) {
	case 1:
		__asm__ __volatile__ ("casb %0,%2,%1"
				      : "=d" (old), "=m" (*(char *)p)
				      : "d" (new), "0" (old), "m" (*(char *)p));
		break;
	case 2:
		__asm__ __volatile__ ("casw %0,%2,%1"
				      : "=d" (old), "=m" (*(short *)p)
				      : "d" (new), "0" (old), "m" (*(short *)p));
		break;
	case 4:
		__asm__ __volatile__ ("casl %0,%2,%1"
				      : "=d" (old), "=m" (*(int *)p)
				      : "d" (new), "0" (old), "m" (*(int *)p));
		break;
	}
	return old;
}

#define cmpxchg(ptr, o, n)						    \
	((__typeof__(*(ptr)))__cmpxchg((ptr), (unsigned long)(o),	    \
			(unsigned long)(n), sizeof(*(ptr))))
#define cmpxchg_local(ptr, o, n)					    \
	((__typeof__(*(ptr)))__cmpxchg((ptr), (unsigned long)(o),	    \
			(unsigned long)(n), sizeof(*(ptr))))
#else

#define cmpxchg_local(ptr, o, n)				  	       \
	((__typeof__(*(ptr)))__cmpxchg_local_generic((ptr), (unsigned long)(o),\
			(unsigned long)(n), sizeof(*(ptr))))

#ifndef CONFIG_SMP
#include <asm-generic/cmpxchg.h>
#endif

#endif

#define arch_align_stack(x) (x)

#endif /* __KERNEL__ */

#endif /* _M68K_SYSTEM_H */
