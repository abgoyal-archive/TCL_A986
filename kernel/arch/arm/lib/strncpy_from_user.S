
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/errno.h>

	.text
	.align	5

ENTRY(__strncpy_from_user)
	mov	ip, r1
1:	subs	r2, r2, #1
	ldrusr	r3, r1, 1, pl
	bmi	2f
	strb	r3, [r0], #1
	teq	r3, #0
	bne	1b
	sub	r1, r1, #1	@ take NUL character out of count
2:	sub	r0, r1, ip
	mov	pc, lr
ENDPROC(__strncpy_from_user)

	.pushsection .fixup,"ax"
	.align	0
9001:	mov	r3, #0
	strb	r3, [r0, #0]	@ null terminate
	mov	r0, #-EFAULT
	mov	pc, lr
	.popsection

