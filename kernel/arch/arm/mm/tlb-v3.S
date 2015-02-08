
#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/asm-offsets.h>
#include <asm/tlbflush.h>
#include "proc-macros.S"

	.align	5
	.align	5
ENTRY(v3_flush_user_tlb_range)
	vma_vm_mm r2, r2
	act_mm	r3				@ get current->active_mm
	teq	r2, r3				@ == mm ?
	movne	pc, lr				@ no, we dont do anything
ENTRY(v3_flush_kern_tlb_range)
	bic	r0, r0, #0x0ff
	bic	r0, r0, #0xf00
1:	mcr	p15, 0, r0, c6, c0, 0		@ invalidate TLB entry
	add	r0, r0, #PAGE_SZ
	cmp	r0, r1
	blo	1b
	mov	pc, lr

	__INITDATA

	.type	v3_tlb_fns, #object
ENTRY(v3_tlb_fns)
	.long	v3_flush_user_tlb_range
	.long	v3_flush_kern_tlb_range
	.long	v3_tlb_flags
	.size	v3_tlb_fns, . - v3_tlb_fns
