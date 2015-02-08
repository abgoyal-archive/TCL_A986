
#ifndef __ASM_SH_SYSTEM_64_H
#define __ASM_SH_SYSTEM_64_H

#include <cpu/registers.h>
#include <asm/processor.h>

struct thread_struct;
struct task_struct *sh64_switch_to(struct task_struct *prev,
				   struct thread_struct *prev_thread,
				   struct task_struct *next,
				   struct thread_struct *next_thread);

#define switch_to(prev,next,last)				\
do {								\
	if (last_task_used_math != next) {			\
		struct pt_regs *regs = next->thread.uregs;	\
		if (regs) regs->sr |= SR_FD;			\
	}							\
	last = sh64_switch_to(prev, &prev->thread, next,	\
			      &next->thread);			\
} while (0)

#define jump_to_uncached()	do { } while (0)
#define back_to_cached()	do { } while (0)

#define __icbi(addr)	__asm__ __volatile__ ( "icbi %0, 0\n\t" : : "r" (addr))
#define __ocbp(addr)	__asm__ __volatile__ ( "ocbp %0, 0\n\t" : : "r" (addr))
#define __ocbi(addr)	__asm__ __volatile__ ( "ocbi %0, 0\n\t" : : "r" (addr))
#define __ocbwb(addr)	__asm__ __volatile__ ( "ocbwb %0, 0\n\t" : : "r" (addr))

static inline reg_size_t register_align(void *val)
{
	return (unsigned long long)(signed long long)(signed long)val;
}

extern void phys_stext(void);

static inline void trigger_address_error(void)
{
	phys_stext();
}

#define SR_BL_LL	0x0000000010000000LL

static inline void set_bl_bit(void)
{
	unsigned long long __dummy0, __dummy1 = SR_BL_LL;

	__asm__ __volatile__("getcon	" __SR ", %0\n\t"
			     "or	%0, %1, %0\n\t"
			     "putcon	%0, " __SR "\n\t"
			     : "=&r" (__dummy0)
			     : "r" (__dummy1));

}

static inline void clear_bl_bit(void)
{
	unsigned long long __dummy0, __dummy1 = ~SR_BL_LL;

	__asm__ __volatile__("getcon	" __SR ", %0\n\t"
			     "and	%0, %1, %0\n\t"
			     "putcon	%0, " __SR "\n\t"
			     : "=&r" (__dummy0)
			     : "r" (__dummy1));
}

#endif /* __ASM_SH_SYSTEM_64_H */
