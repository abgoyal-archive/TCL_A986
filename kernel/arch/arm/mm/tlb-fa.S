
#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/asm-offsets.h>
#include <asm/tlbflush.h>
#include "proc-macros.S"


	.align	4
ENTRY(fa_flush_user_tlb_range)
	vma_vm_mm ip, r2
	act_mm	r3				@ get current->active_mm
	eors	r3, ip, r3			@ == mm ?
	movne	pc, lr				@ no, we dont do anything
	mov	r3, #0
	mcr	p15, 0, r3, c7, c10, 4		@ drain WB
	bic	r0, r0, #0x0ff
	bic	r0, r0, #0xf00
1:	mcr	p15, 0, r0, c8, c7, 1		@ invalidate UTLB entry
	add	r0, r0, #PAGE_SZ
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r3, c7, c5, 6		@ invalidate BTB
	mcr	p15, 0, r3, c7, c10, 4		@ data write barrier
	mov	pc, lr


ENTRY(fa_flush_kern_tlb_range)
	mov	r3, #0
	mcr	p15, 0, r3, c7, c10, 4		@ drain WB
	bic	r0, r0, #0x0ff
	bic	r0, r0, #0xf00
1:	mcr	p15, 0, r0, c8, c7, 1		@ invalidate UTLB entry
	add	r0, r0, #PAGE_SZ
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r3, c7, c5, 6		@ invalidate BTB
	mcr	p15, 0, r3, c7, c10, 4		@ data write barrier
	mcr	p15, 0, r3, c7, c5, 4		@ prefetch flush
	mov	pc, lr

	__INITDATA

	.type	fa_tlb_fns, #object
ENTRY(fa_tlb_fns)
	.long	fa_flush_user_tlb_range
	.long	fa_flush_kern_tlb_range
	.long	fa_tlb_flags
	.size	fa_tlb_fns, . - fa_tlb_fns
