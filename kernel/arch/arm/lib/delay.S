
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/param.h>
		.text

.LC0:		.word	loops_per_jiffy
.LC1:		.word	(2199023*HZ)>>11


ENTRY(__udelay)
		ldr	r2, .LC1
		mul	r0, r2, r0
ENTRY(__const_udelay)				@ 0 <= r0 <= 0x7fffff06
		ldr	r2, .LC0
		ldr	r2, [r2]		@ max = 0x01ffffff
		mov	r0, r0, lsr #14		@ max = 0x0001ffff
		mov	r2, r2, lsr #10		@ max = 0x00007fff
		mul	r0, r2, r0		@ max = 2^32-1
		movs	r0, r0, lsr #6
		moveq	pc, lr


@ Delay routine
ENTRY(__delay)
		subs	r0, r0, #1
#if 0
		movls	pc, lr
		subs	r0, r0, #1
		movls	pc, lr
		subs	r0, r0, #1
		movls	pc, lr
		subs	r0, r0, #1
		movls	pc, lr
		subs	r0, r0, #1
		movls	pc, lr
		subs	r0, r0, #1
		movls	pc, lr
		subs	r0, r0, #1
		movls	pc, lr
		subs	r0, r0, #1
#endif
		bhi	__delay
		mov	pc, lr
ENDPROC(__udelay)
ENDPROC(__const_udelay)
ENDPROC(__delay)
