
#include <linux/linkage.h>
#include <asm/assembler.h>

		.text


		.macro	save_regs
		stmfd	sp!, {r1, r4 - r8, lr}
		.endm

		.macro	load_regs
		ldmfd	sp!, {r1, r4 - r8, pc}
		.endm

		.macro	load1b, reg1
		ldrb	\reg1, [r0], #1
		.endm

		.macro	load2b, reg1, reg2
		ldrb	\reg1, [r0], #1
		ldrb	\reg2, [r0], #1
		.endm

		.macro	load1l, reg1
		ldr	\reg1, [r0], #4
		.endm

		.macro	load2l, reg1, reg2
		ldr	\reg1, [r0], #4
		ldr	\reg2, [r0], #4
		.endm

		.macro	load4l, reg1, reg2, reg3, reg4
		ldmia	r0!, {\reg1, \reg2, \reg3, \reg4}
		.endm

#define FN_ENTRY	ENTRY(csum_partial_copy_nocheck)
#define FN_EXIT		ENDPROC(csum_partial_copy_nocheck)

#include "csumpartialcopygeneric.S"
