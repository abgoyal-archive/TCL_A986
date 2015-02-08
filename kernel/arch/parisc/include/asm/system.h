
#ifndef __PARISC_SYSTEM_H
#define __PARISC_SYSTEM_H

#include <asm/psw.h>

/* The program status word as bitfields.  */
struct pa_psw {
	unsigned int y:1;
	unsigned int z:1;
	unsigned int rv:2;
	unsigned int w:1;
	unsigned int e:1;
	unsigned int s:1;
	unsigned int t:1;

	unsigned int h:1;
	unsigned int l:1;
	unsigned int n:1;
	unsigned int x:1;
	unsigned int b:1;
	unsigned int c:1;
	unsigned int v:1;
	unsigned int m:1;

	unsigned int cb:8;

	unsigned int o:1;
	unsigned int g:1;
	unsigned int f:1;
	unsigned int r:1;
	unsigned int q:1;
	unsigned int p:1;
	unsigned int d:1;
	unsigned int i:1;
};

#ifdef CONFIG_64BIT
#define pa_psw(task) ((struct pa_psw *) ((char *) (task) + TASK_PT_PSW + 4))
#else
#define pa_psw(task) ((struct pa_psw *) ((char *) (task) + TASK_PT_PSW))
#endif

struct task_struct;

extern struct task_struct *_switch_to(struct task_struct *, struct task_struct *);

#define switch_to(prev, next, last) do {			\
	(last) = _switch_to(prev, next);			\
} while(0)

/* interrupt control */
#define local_save_flags(x)	__asm__ __volatile__("ssm 0, %0" : "=r" (x) : : "memory")
#define local_irq_disable()	__asm__ __volatile__("rsm %0,%%r0\n" : : "i" (PSW_I) : "memory" )
#define local_irq_enable()	__asm__ __volatile__("ssm %0,%%r0\n" : : "i" (PSW_I) : "memory" )

#define local_irq_save(x) \
	__asm__ __volatile__("rsm %1,%0" : "=r" (x) :"i" (PSW_I) : "memory" )
#define local_irq_restore(x) \
	__asm__ __volatile__("mtsm %0" : : "r" (x) : "memory" )

#define irqs_disabled()			\
({					\
	unsigned long flags;		\
	local_save_flags(flags);	\
	(flags & PSW_I) == 0;		\
})

#define mfctl(reg)	({		\
	unsigned long cr;		\
	__asm__ __volatile__(		\
		"mfctl " #reg ",%0" :	\
		 "=r" (cr)		\
	);				\
	cr;				\
})

#define mtctl(gr, cr) \
	__asm__ __volatile__("mtctl %0,%1" \
		: /* no outputs */ \
		: "r" (gr), "i" (cr) : "memory")

/* these are here to de-mystefy the calling code, and to provide hooks */
/* which I needed for debugging EIEM problems -PB */
#define get_eiem() mfctl(15)
static inline void set_eiem(unsigned long val)
{
	mtctl(val, 15);
}

#define mfsp(reg)	({		\
	unsigned long cr;		\
	__asm__ __volatile__(		\
		"mfsp " #reg ",%0" :	\
		 "=r" (cr)		\
	);				\
	cr;				\
})

#define mtsp(gr, cr) \
	__asm__ __volatile__("mtsp %0,%1" \
		: /* no outputs */ \
		: "r" (gr), "i" (cr) : "memory")


#define mb()		__asm__ __volatile__("":::"memory")	/* barrier() */
#define rmb()		mb()
#define wmb()		mb()
#define smp_mb()	mb()
#define smp_rmb()	mb()
#define smp_wmb()	mb()
#define smp_read_barrier_depends()	do { } while(0)
#define read_barrier_depends()		do { } while(0)

#define set_mb(var, value)		do { var = value; mb(); } while (0)

#ifndef CONFIG_PA20

#define __PA_LDCW_ALIGNMENT	16
#define __ldcw_align(a) ({					\
	unsigned long __ret = (unsigned long) &(a)->lock[0];	\
	__ret = (__ret + __PA_LDCW_ALIGNMENT - 1)		\
		& ~(__PA_LDCW_ALIGNMENT - 1);			\
	(volatile unsigned int *) __ret;			\
})
#define __LDCW	"ldcw"

#else /*CONFIG_PA20*/

#define __PA_LDCW_ALIGNMENT	4
#define __ldcw_align(a) (&(a)->slock)
#define __LDCW	"ldcw,co"

#endif /*!CONFIG_PA20*/

/* LDCW, the only atomic read-write operation PA-RISC has. *sigh*.  */
#define __ldcw(a) ({						\
	unsigned __ret;						\
	__asm__ __volatile__(__LDCW " 0(%2),%0"			\
		: "=r" (__ret), "+m" (*(a)) : "r" (a));		\
	__ret;							\
})

#ifdef CONFIG_SMP
# define __lock_aligned __attribute__((__section__(".data..lock_aligned")))
#endif

#define arch_align_stack(x) (x)

#endif
