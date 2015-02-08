
#include <linux/linkage.h>
#include <asm/assembler.h>
#include "abort-macro.S"
	.align	5
ENTRY(v5tj_early_abort)
	mrc	p15, 0, r1, c5, c0, 0		@ get FSR
	mrc	p15, 0, r0, c6, c0, 0		@ get FAR
	bic	r1, r1, #1 << 11 | 1 << 10	@ clear bits 11 and 10 of FSR
	tst	r3, #PSR_J_BIT			@ Java?
	movne	pc, lr
	do_thumb_abort
	ldreq	r3, [r2]			@ read aborted ARM instruction
	do_ldrd_abort
	tst	r3, #1 << 20			@ L = 0 -> write
	orreq	r1, r1, #1 << 11		@ yes.
	mov	pc, lr


