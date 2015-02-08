
#ifndef _H8300_SYSTEM_H
#define _H8300_SYSTEM_H

#include <linux/linkage.h>


asmlinkage void resume(void);
#define switch_to(prev,next,last) {                         \
  void *_last;						    \
  __asm__ __volatile__(					    \
  			"mov.l	%1, er0\n\t"		    \
			"mov.l	%2, er1\n\t"		    \
                        "mov.l  %3, er2\n\t"                \
			"jsr @_resume\n\t"                  \
                        "mov.l  er2,%0\n\t"                 \
		       : "=r" (_last)			    \
		       : "r" (&(prev->thread)),		    \
			 "r" (&(next->thread)),		    \
                         "g" (prev)                         \
		       : "cc", "er0", "er1", "er2", "er3"); \
  (last) = _last; 					    \
}

#define __sti() asm volatile ("andc #0x7f,ccr")
#define __cli() asm volatile ("orc  #0x80,ccr")

#define __save_flags(x) \
       asm volatile ("stc ccr,%w0":"=r" (x))

#define __restore_flags(x) \
       asm volatile ("ldc %w0,ccr": :"r" (x))

#define	irqs_disabled()			\
({					\
	unsigned char flags;		\
	__save_flags(flags);	        \
	((flags & 0x80) == 0x80);	\
})

#define iret() __asm__ __volatile__ ("rte": : :"memory", "sp", "cc")

/* For spinlocks etc */
#define local_irq_disable()	__cli()
#define local_irq_enable()      __sti()
#define local_irq_save(x)	({ __save_flags(x); local_irq_disable(); })
#define local_irq_restore(x)	__restore_flags(x)
#define local_save_flags(x)     __save_flags(x)

#define nop()  asm volatile ("nop"::)
#define mb()   asm volatile (""   : : :"memory")
#define rmb()  asm volatile (""   : : :"memory")
#define wmb()  asm volatile (""   : : :"memory")
#define set_mb(var, value) do { xchg(&var, value); } while (0)

#ifdef CONFIG_SMP
#define smp_mb()	mb()
#define smp_rmb()	rmb()
#define smp_wmb()	wmb()
#define smp_read_barrier_depends()	read_barrier_depends()
#else
#define smp_mb()	barrier()
#define smp_rmb()	barrier()
#define smp_wmb()	barrier()
#define smp_read_barrier_depends()	do { } while(0)
#endif

#define xchg(ptr,x) ((__typeof__(*(ptr)))__xchg((unsigned long)(x),(ptr),sizeof(*(ptr))))

struct __xchg_dummy { unsigned long a[100]; };
#define __xg(x) ((volatile struct __xchg_dummy *)(x))

static inline unsigned long __xchg(unsigned long x, volatile void * ptr, int size)
{
  unsigned long tmp, flags;

  local_irq_save(flags);

  switch (size) {
  case 1:
    __asm__ __volatile__
    ("mov.b %2,%0\n\t"
     "mov.b %1,%2"
    : "=&r" (tmp) : "r" (x), "m" (*__xg(ptr)) : "memory");
    break;
  case 2:
    __asm__ __volatile__
    ("mov.w %2,%0\n\t"
     "mov.w %1,%2"
    : "=&r" (tmp) : "r" (x), "m" (*__xg(ptr)) : "memory");
    break;
  case 4:
    __asm__ __volatile__
    ("mov.l %2,%0\n\t"
     "mov.l %1,%2"
    : "=&r" (tmp) : "r" (x), "m" (*__xg(ptr)) : "memory");
    break;
  default:
    tmp = 0;	  
  }
  local_irq_restore(flags);
  return tmp;
}

#define HARD_RESET_NOW() ({		\
        local_irq_disable();		\
        asm("jmp @@0");			\
})

#include <asm-generic/cmpxchg-local.h>

#define cmpxchg_local(ptr, o, n)				  	       \
	((__typeof__(*(ptr)))__cmpxchg_local_generic((ptr), (unsigned long)(o),\
			(unsigned long)(n), sizeof(*(ptr))))
#define cmpxchg64_local(ptr, o, n) __cmpxchg64_local_generic((ptr), (o), (n))

#ifndef CONFIG_SMP
#include <asm-generic/cmpxchg.h>
#endif

#define arch_align_stack(x) (x)

void die(char *str, struct pt_regs *fp, unsigned long err);

#endif /* _H8300_SYSTEM_H */
