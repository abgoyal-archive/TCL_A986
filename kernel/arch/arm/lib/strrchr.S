
#include <linux/linkage.h>
#include <asm/assembler.h>

		.text
		.align	5
ENTRY(strrchr)
		mov	r3, #0
1:		ldrb	r2, [r0], #1
		teq	r2, r1
		subeq	r3, r0, #1
		teq	r2, #0
		bne	1b
		mov	r0, r3
		mov	pc, lr
ENDPROC(strrchr)
