
#include <linux/linkage.h>
#include <asm/assembler.h>
#include "abort-macro.S"
	.align	5
ENTRY(v6_early_abort)
#ifdef CONFIG_CPU_32v6K
	clrex
#else
	sub	r1, sp, #4			@ Get unused stack location
	strex	r0, r1, [r1]			@ Clear the exclusive monitor
#endif
	mrc	p15, 0, r1, c5, c0, 0		@ get FSR
	mrc	p15, 0, r0, c6, c0, 0		@ get FAR
	bic	r1, r1, #1 << 11		@ clear bit 11 of FSR
	tst	r3, #PSR_J_BIT			@ Java?
	movne	pc, lr
	do_thumb_abort
	ldreq	r3, [r2]			@ read aborted ARM instruction
#ifdef CONFIG_CPU_ENDIAN_BE8
	reveq	r3, r3
#endif
	do_ldrd_abort
	tst	r3, #1 << 20			@ L = 0 -> write
	orreq	r1, r1, #1 << 11		@ yes.
	mov	pc, lr


