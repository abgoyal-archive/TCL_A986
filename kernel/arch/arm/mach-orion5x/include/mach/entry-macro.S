

#include <mach/bridge-regs.h>

	.macro  disable_fiq
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm

	.macro  get_irqnr_preamble, base, tmp
	ldr	\base, =MAIN_IRQ_CAUSE
	.endm

	.macro  get_irqnr_and_base, irqnr, irqstat, base, tmp
	ldr	\irqstat, [\base, #0]		@ main cause
	ldr	\tmp, [\base, #(MAIN_IRQ_MASK - MAIN_IRQ_CAUSE)] @ main mask
	mov	\irqnr, #0			@ default irqnr
	@ find cause bits that are unmasked
	ands	\irqstat, \irqstat, \tmp	@ clear Z flag if any
	clzne	\irqnr,	\irqstat		@ calc irqnr
	rsbne	\irqnr, \irqnr, #31
	.endm
