

#include <mach/bridge-regs.h>

	.macro  disable_fiq
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm

	.macro  get_irqnr_preamble, base, tmp
	ldr	\base, =IRQ_VIRT_BASE
	.endm

	.macro  get_irqnr_and_base, irqnr, irqstat, base, tmp
	ldr	\irqstat, [\base, #IRQ_CAUSE_OFF]
	ldr	\tmp, [\base, #IRQ_MASK_OFF]
	mov	\irqnr, #0
	ands	\irqstat, \irqstat, \tmp
	clzne	\irqnr, \irqstat
	rsbne	\irqnr, \irqnr, #31
	.endm
