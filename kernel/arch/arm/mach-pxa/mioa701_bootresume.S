

#include <linux/linkage.h>
#include <asm/assembler.h>

	.data
ENTRY(mioa701_bootstrap)
0:
	b	1f
ENTRY(mioa701_jumpaddr)
	.word	0x40f00008		@ PSPR in no-MMU mode
1:
	mov	r0,     #0xa0000000	@ Don't suppose memory access works
	orr	r0, r0, #0x00200000	@ even if it's supposed to
	orr	r0, r0, #0x0000b000
	mov	r1, #0
	str	r1, [r0]		@ Early disable resume for next boot
	ldr	r0, mioa701_jumpaddr	@ (Murphy's Law)
	ldr	r0, [r0]
	mov	pc, r0
2:

ENTRY(mioa701_bootstrap_lg)
	.data
	.word	2b-0b
