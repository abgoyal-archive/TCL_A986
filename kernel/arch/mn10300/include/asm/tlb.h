

#ifndef _ASM_TLB_H
#define _ASM_TLB_H

#include <asm/tlbflush.h>

extern void check_pgt_cache(void);

#define tlb_start_vma(tlb, vma)				do { } while (0)
#define tlb_end_vma(tlb, vma)				do { } while (0)
#define __tlb_remove_tlb_entry(tlb, ptep, address)	do { } while (0)

#define tlb_flush(tlb)	flush_tlb_mm((tlb)->mm)

/* for now, just use the generic stuff */
#include <asm-generic/tlb.h>

#endif /* _ASM_TLB_H */
