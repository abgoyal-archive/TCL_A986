
#include <linux/linkage.h>
#include <asm/assembler.h>
	.align	5
ENTRY(nommu_early_abort)
	mov	r0, #0				@ clear r0, r1 (no FSR/FAR)
	mov	r1, #0
	mov	pc, lr
ENDPROC(nommu_early_abort)
