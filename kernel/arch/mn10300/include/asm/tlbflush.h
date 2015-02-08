
#ifndef _ASM_TLBFLUSH_H
#define _ASM_TLBFLUSH_H

#include <asm/processor.h>

#define __flush_tlb()						\
do {								\
	int w;							\
	__asm__ __volatile__					\
		("	mov %1,%0		\n"		\
		 "	or %2,%0		\n"		\
		 "	mov %0,%1		\n"		\
		 : "=d"(w)					\
		 : "m"(MMUCTR), "i"(MMUCTR_IIV|MMUCTR_DIV)	\
		 : "cc", "memory"				\
		 );						\
} while (0)

#define __flush_tlb_all() __flush_tlb()
#define __flush_tlb_one(addr) __flush_tlb()


#define flush_tlb_all()				\
do {						\
	preempt_disable();			\
	__flush_tlb_all();			\
	preempt_enable();			\
} while (0)

#define flush_tlb_mm(mm)			\
do {						\
	preempt_disable();			\
	__flush_tlb_all();			\
	preempt_enable();			\
} while (0)

#define flush_tlb_range(vma, start, end)			\
do {								\
	unsigned long __s __attribute__((unused)) = (start);	\
	unsigned long __e __attribute__((unused)) = (end);	\
	preempt_disable();					\
	__flush_tlb_all();					\
	preempt_enable();					\
} while (0)


#define __flush_tlb_global()			flush_tlb_all()
#define flush_tlb()				flush_tlb_all()
#define flush_tlb_kernel_range(start, end)			\
do {								\
	unsigned long __s __attribute__((unused)) = (start);	\
	unsigned long __e __attribute__((unused)) = (end);	\
	flush_tlb_all();					\
} while (0)

extern void flush_tlb_page(struct vm_area_struct *vma, unsigned long addr);

#define flush_tlb_pgtables(mm, start, end)	do {} while (0)

#endif /* _ASM_TLBFLUSH_H */
