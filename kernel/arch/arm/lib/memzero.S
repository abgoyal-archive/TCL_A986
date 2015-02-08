
#include <linux/linkage.h>
#include <asm/assembler.h>

	.text
	.align	5
	.word	0
1:	subs	r1, r1, #4		@ 1 do we have enough
	blt	5f			@ 1 bytes to align with?
	cmp	r3, #2			@ 1
	strltb	r2, [r0], #1		@ 1
	strleb	r2, [r0], #1		@ 1
	strb	r2, [r0], #1		@ 1
	add	r1, r1, r3		@ 1 (r1 = r1 - (4 - r3))

ENTRY(__memzero)
	mov	r2, #0			@ 1
	ands	r3, r0, #3		@ 1 unaligned?
	bne	1b			@ 1
	cmp	r1, #16			@ 1 we can skip this chunk if we
	blt	4f			@ 1 have < 16 bytes

#if ! CALGN(1)+0

	str	lr, [sp, #-4]!		@ 1
	mov	ip, r2			@ 1
	mov	lr, r2			@ 1

3:	subs	r1, r1, #64		@ 1 write 32 bytes out per loop
	stmgeia	r0!, {r2, r3, ip, lr}	@ 4
	stmgeia	r0!, {r2, r3, ip, lr}	@ 4
	stmgeia	r0!, {r2, r3, ip, lr}	@ 4
	stmgeia	r0!, {r2, r3, ip, lr}	@ 4
	bgt	3b			@ 1
	ldmeqfd	sp!, {pc}		@ 1/2 quick exit
	tst	r1, #32			@ 1
	stmneia	r0!, {r2, r3, ip, lr}	@ 4
	stmneia	r0!, {r2, r3, ip, lr}	@ 4
	tst	r1, #16			@ 1 16 bytes or more?
	stmneia	r0!, {r2, r3, ip, lr}	@ 4
	ldr	lr, [sp], #4		@ 1

#else


	stmfd	sp!, {r4-r7, lr}
	mov	r4, r2
	mov	r5, r2
	mov	r6, r2
	mov	r7, r2
	mov	ip, r2
	mov	lr, r2

	cmp	r1, #96
	andgts	ip, r0, #31
	ble	3f

	rsb	ip, ip, #32
	sub	r1, r1, ip
	movs	ip, ip, lsl #(32 - 4)
	stmcsia	r0!, {r4, r5, r6, r7}
	stmmiia	r0!, {r4, r5}
	movs	ip, ip, lsl #2
	strcs	r2, [r0], #4

3:	subs	r1, r1, #64
	stmgeia	r0!, {r2-r7, ip, lr}
	stmgeia	r0!, {r2-r7, ip, lr}
	bgt	3b
	ldmeqfd	sp!, {r4-r7, pc}

	tst	r1, #32
	stmneia	r0!, {r2-r7, ip, lr}
	tst	r1, #16
	stmneia	r0!, {r4-r7}
	ldmfd	sp!, {r4-r7, lr}

#endif

4:	tst	r1, #8			@ 1 8 bytes or more?
	stmneia	r0!, {r2, r3}		@ 2
	tst	r1, #4			@ 1 4 bytes or more?
	strne	r2, [r0], #4		@ 1
5:	tst	r1, #2			@ 1 2 bytes or more?
	strneb	r2, [r0], #1		@ 1
	strneb	r2, [r0], #1		@ 1
	tst	r1, #1			@ 1 a byte left over
	strneb	r2, [r0], #1		@ 1
	mov	pc, lr			@ 1
ENDPROC(__memzero)
