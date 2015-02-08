
#include <linux/linkage.h>
#include <asm/errno.h>

ENTRY(__get_user_1)
1:	ldrbt	r2, [r0]
	mov	r0, #0
	mov	pc, lr
ENDPROC(__get_user_1)

ENTRY(__get_user_2)
#ifdef CONFIG_THUMB2_KERNEL
2:	ldrbt	r2, [r0]
3:	ldrbt	r3, [r0, #1]
#else
2:	ldrbt	r2, [r0], #1
3:	ldrbt	r3, [r0]
#endif
#ifndef __ARMEB__
	orr	r2, r2, r3, lsl #8
#else
	orr	r2, r3, r2, lsl #8
#endif
	mov	r0, #0
	mov	pc, lr
ENDPROC(__get_user_2)

ENTRY(__get_user_4)
4:	ldrt	r2, [r0]
	mov	r0, #0
	mov	pc, lr
ENDPROC(__get_user_4)

__get_user_bad:
	mov	r2, #0
	mov	r0, #-EFAULT
	mov	pc, lr
ENDPROC(__get_user_bad)

.pushsection __ex_table, "a"
	.long	1b, __get_user_bad
	.long	2b, __get_user_bad
	.long	3b, __get_user_bad
	.long	4b, __get_user_bad
.popsection
