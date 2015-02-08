
#ifndef __ASM_AVR32_TLBFLUSH_H
#define __ASM_AVR32_TLBFLUSH_H

#include <asm/mmu.h>

extern void flush_tlb(void);
extern void flush_tlb_all(void);
extern void flush_tlb_mm(struct mm_struct *mm);
extern void flush_tlb_range(struct vm_area_struct *vma, unsigned long start,
			    unsigned long end);
extern void flush_tlb_page(struct vm_area_struct *vma, unsigned long page);

extern void flush_tlb_kernel_range(unsigned long start, unsigned long end);

#endif /* __ASM_AVR32_TLBFLUSH_H */
