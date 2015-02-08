
#ifndef _ASM_SYSTEM_H
#define _ASM_SYSTEM_H

#include <asm/cpu-regs.h>

#ifdef __KERNEL__
#ifndef __ASSEMBLY__

#include <linux/kernel.h>

struct task_struct;
struct thread_struct;

extern asmlinkage
struct task_struct *__switch_to(struct thread_struct *prev,
				struct thread_struct *next,
				struct task_struct *prev_task);

/* context switching is now performed out-of-line in switch_to.S */
#define switch_to(prev, next, last)					\
do {									\
	current->thread.wchan = (u_long) __builtin_return_address(0);	\
	(last) = __switch_to(&(prev)->thread, &(next)->thread, (prev));	\
	mb();								\
	current->thread.wchan = 0;					\
} while (0)

#define arch_align_stack(x) (x)

#define nop() asm volatile ("nop")

#endif /* !__ASSEMBLY__ */


#define mb()	asm volatile ("": : :"memory")
#define rmb()	mb()
#define wmb()	asm volatile ("": : :"memory")

#ifdef CONFIG_SMP
#define smp_mb()	mb()
#define smp_rmb()	rmb()
#define smp_wmb()	wmb()
#else
#define smp_mb()	barrier()
#define smp_rmb()	barrier()
#define smp_wmb()	barrier()
#endif

#define set_mb(var, value)  do { var = value;  mb(); } while (0)
#define set_wmb(var, value) do { var = value; wmb(); } while (0)

#define read_barrier_depends()		do {} while (0)
#define smp_read_barrier_depends()	do {} while (0)

/*****************************************************************************/
#ifdef CONFIG_MN10300_TTYSM
#define MN10300_CLI_LEVEL	EPSW_IM_2
#else
#define MN10300_CLI_LEVEL	EPSW_IM_1
#endif

#define local_save_flags(x)			\
do {						\
	typecheck(unsigned long, x);		\
	asm volatile(				\
		"	mov epsw,%0	\n"	\
		: "=d"(x)			\
		);				\
} while (0)

#define local_irq_disable()						\
do {									\
	asm volatile(							\
		"	and %0,epsw	\n"				\
		"	or %1,epsw	\n"				\
		"	nop		\n"				\
		"	nop		\n"				\
		"	nop		\n"				\
		:							\
		: "i"(~EPSW_IM), "i"(EPSW_IE | MN10300_CLI_LEVEL)	\
		);							\
} while (0)

#define local_irq_save(x)			\
do {						\
	local_save_flags(x);			\
	local_irq_disable();			\
} while (0)

#ifndef __ASSEMBLY__

extern unsigned long __mn10300_irq_enabled_epsw;

#endif

#define local_irq_enable()						\
do {									\
	unsigned long tmp;						\
									\
	asm volatile(							\
		"	mov	epsw,%0		\n"			\
		"	and	%1,%0		\n"			\
		"	or	%2,%0		\n"			\
		"	mov	%0,epsw		\n"			\
		: "=&d"(tmp)						\
		: "i"(~EPSW_IM), "r"(__mn10300_irq_enabled_epsw)	\
		: "cc"							\
		);							\
} while (0)

#define local_irq_restore(x)			\
do {						\
	typecheck(unsigned long, x);		\
	asm volatile(				\
		"	mov %0,epsw	\n"	\
		"	nop		\n"	\
		"	nop		\n"	\
		"	nop		\n"	\
		:				\
		: "d"(x)			\
		: "memory", "cc"		\
		);				\
} while (0)

#define irqs_disabled()				\
({						\
	unsigned long flags;			\
	local_save_flags(flags);		\
	(flags & EPSW_IM) <= MN10300_CLI_LEVEL;	\
})

#define safe_halt()							\
do {									\
	asm volatile("	or	%0,epsw	\n"				\
		     "	nop		\n"				\
		     "	nop		\n"				\
		     "	bset	%2,(%1)	\n"				\
		     :							\
		     : "i"(EPSW_IE|EPSW_IM), "n"(&CPUM), "i"(CPUM_SLEEP)\
		     : "cc"						\
		     );							\
} while (0)

#define STI	or EPSW_IE|EPSW_IM,epsw
#define CLI	and ~EPSW_IM,epsw; or EPSW_IE|MN10300_CLI_LEVEL,epsw; nop; nop; nop

/*****************************************************************************/
#ifndef __ASSEMBLY__

struct __xchg_dummy { unsigned long a[100]; };
#define __xg(x) ((struct __xchg_dummy *)(x))

static inline
unsigned long __xchg(volatile unsigned long *m, unsigned long val)
{
	unsigned long retval;
	unsigned long flags;

	local_irq_save(flags);
	retval = *m;
	*m = val;
	local_irq_restore(flags);
	return retval;
}

#define xchg(ptr, v)						\
	((__typeof__(*(ptr))) __xchg((unsigned long *)(ptr),	\
				     (unsigned long)(v)))

static inline unsigned long __cmpxchg(volatile unsigned long *m,
				      unsigned long old, unsigned long new)
{
	unsigned long retval;
	unsigned long flags;

	local_irq_save(flags);
	retval = *m;
	if (retval == old)
		*m = new;
	local_irq_restore(flags);
	return retval;
}

#define cmpxchg(ptr, o, n)					\
	((__typeof__(*(ptr))) __cmpxchg((unsigned long *)(ptr), \
					(unsigned long)(o),	\
					(unsigned long)(n)))

#endif /* !__ASSEMBLY__ */

#endif /* __KERNEL__ */
#endif /* _ASM_SYSTEM_H */
