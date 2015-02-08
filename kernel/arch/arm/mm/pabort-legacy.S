
#include <linux/linkage.h>
#include <asm/assembler.h>


	.align	5
ENTRY(legacy_pabort)
	mov	r1, #5
	mov	pc, lr
ENDPROC(legacy_pabort)
