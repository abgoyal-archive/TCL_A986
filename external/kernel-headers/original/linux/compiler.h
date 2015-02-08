#ifndef __LINUX_COMPILER_H
#define __LINUX_COMPILER_H

#ifndef __ASSEMBLY__

#ifdef __CHECKER__
# define __user		__attribute__((noderef, address_space(1)))
# define __kernel	/* default address space */
# define __safe		__attribute__((safe))
# define __force	__attribute__((force))
# define __nocast	__attribute__((nocast))
# define __iomem	__attribute__((noderef, address_space(2)))
# define __acquires(x)	__attribute__((context(0,1)))
# define __releases(x)	__attribute__((context(1,0)))
# define __acquire(x)	__context__(1)
# define __release(x)	__context__(-1)
# define __cond_lock(x)	((x) ? ({ __context__(1); 1; }) : 0)
extern void __chk_user_ptr(void __user *);
extern void __chk_io_ptr(void __iomem *);
#else
# define __user
# define __kernel
# define __safe
# define __force
# define __nocast
# define __iomem
# define __chk_user_ptr(x) (void)0
# define __chk_io_ptr(x) (void)0
# define __builtin_warning(x, y...) (1)
# define __acquires(x)
# define __releases(x)
# define __acquire(x) (void)0
# define __release(x) (void)0
# define __cond_lock(x) (x)
#endif

#ifdef __KERNEL__

#if __GNUC__ > 4
#error no compiler-gcc.h file for this gcc version
#elif __GNUC__ == 4
# include <linux/compiler-gcc4.h>
#elif __GNUC__ == 3
# include <linux/compiler-gcc3.h>
#else
# error Sorry, your compiler is too old/not recognized.
#endif

#ifdef __INTEL_COMPILER
# include <linux/compiler-intel.h>
#endif


#define likely(x)	__builtin_expect(!!(x), 1)
#define unlikely(x)	__builtin_expect(!!(x), 0)

/* Optimization barrier */
#ifndef barrier
# define barrier() __memory_barrier()
#endif

#ifndef RELOC_HIDE
# define RELOC_HIDE(ptr, off)					\
  ({ unsigned long __ptr;					\
     __ptr = (unsigned long) (ptr);				\
    (typeof(ptr)) (__ptr + (off)); })
#endif

#endif /* __KERNEL__ */

#endif /* __ASSEMBLY__ */

#ifdef __KERNEL__
#ifndef __deprecated
# define __deprecated		/* unimplemented */
#endif

#ifdef MODULE
#define __deprecated_for_modules __deprecated
#else
#define __deprecated_for_modules
#endif

#ifndef __must_check
#define __must_check
#endif

#ifndef __attribute_used__
# define __attribute_used__	/* unimplemented */
#endif

#ifndef __attribute_pure__
# define __attribute_pure__	/* unimplemented */
#endif

#ifndef noinline
#define noinline
#endif

#ifndef __always_inline
#define __always_inline inline
#endif

#endif /* __KERNEL__ */

#ifndef __attribute_const__
# define __attribute_const__	/* unimplemented */
#endif

#endif /* __LINUX_COMPILER_H */
