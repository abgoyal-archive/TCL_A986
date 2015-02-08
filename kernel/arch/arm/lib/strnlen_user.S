
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/errno.h>

	.text
	.align	5

ENTRY(__strnlen_user)
	mov	r2, r0
1:
	ldrusr	r3, r0, 1
	teq	r3, #0
	beq	2f
	subs	r1, r1, #1
	bne	1b
	add	r0, r0, #1
2:	sub	r0, r0, r2
	mov	pc, lr
ENDPROC(__strnlen_user)

	.pushsection .fixup,"ax"
	.align	0
9001:	mov	r0, #0
	mov	pc, lr
	.popsection
