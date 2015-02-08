
#include <linux/linkage.h>
#include <asm/assembler.h>


	.align	5
ENTRY(v6_pabort)
	mrc	p15, 0, r1, c5, c0, 1		@ get IFSR
	mov	pc, lr
ENDPROC(v6_pabort)
