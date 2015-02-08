
#include <linux/linkage.h>
#include <asm/assembler.h>
#include "abort-macro.S"
	.align	5
ENTRY(v4t_early_abort)
	mrc	p15, 0, r1, c5, c0, 0		@ get FSR
	mrc	p15, 0, r0, c6, c0, 0		@ get FAR
	do_thumb_abort
	ldreq	r3, [r2]			@ read aborted ARM instruction
	bic	r1, r1, #1 << 11 | 1 << 10	@ clear bits 11 and 10 of FSR
	tst	r3, #1 << 20			@ check write
	orreq	r1, r1, #1 << 11
	mov	pc, lr
