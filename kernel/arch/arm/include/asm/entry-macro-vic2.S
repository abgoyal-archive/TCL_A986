


#include <asm/hardware/vic.h>

	.macro	disable_fiq
	.endm

	.macro	get_irqnr_preamble, base, tmp
	ldr	\base, =VA_VIC0
	.endm

	.macro	arch_ret_to_user, tmp1, tmp2
	.endm

	.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp

	@ check the vic0
	mov	\irqnr, #IRQ_VIC0_BASE + 31
	ldr	\irqstat, [ \base, # VIC_IRQ_STATUS ]
	teq	\irqstat, #0

	@ otherwise try vic1
	addeq	\tmp, \base, #(VA_VIC1 - VA_VIC0)
	addeq	\irqnr, \irqnr, #(IRQ_VIC1_BASE - IRQ_VIC0_BASE)
	ldreq	\irqstat, [ \tmp, # VIC_IRQ_STATUS ]
	teqeq	\irqstat, #0

	clzne	\irqstat, \irqstat
	subne	\irqnr, \irqnr, \irqstat
	.endm
