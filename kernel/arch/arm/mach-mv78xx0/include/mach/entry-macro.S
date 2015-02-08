

#include <mach/bridge-regs.h>

	.macro  disable_fiq
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm

	.macro  get_irqnr_preamble, base, tmp
	ldr	\base, =IRQ_VIRT_BASE
	.endm

	.macro  get_irqnr_and_base, irqnr, irqstat, base, tmp
	@ check low interrupts
	ldr	\irqstat, [\base, #IRQ_CAUSE_LOW_OFF]
	ldr	\tmp, [\base, #IRQ_MASK_LOW_OFF]
	mov	\irqnr, #31
	ands	\irqstat, \irqstat, \tmp
	bne	1001f

	@ if no low interrupts set, check high interrupts
	ldr	\irqstat, [\base, #IRQ_CAUSE_HIGH_OFF]
	ldr	\tmp, [\base, #IRQ_MASK_HIGH_OFF]
	mov	\irqnr, #63
	ands	\irqstat, \irqstat, \tmp
	bne	1001f

	@ if no high interrupts set, check error interrupts
	ldr	\irqstat, [\base, #IRQ_CAUSE_ERR_OFF]
	ldr	\tmp, [\base, #IRQ_MASK_ERR_OFF]
	mov	\irqnr, #95
	ands	\irqstat, \irqstat, \tmp

	@ find first active interrupt source
1001:	clzne	\irqstat, \irqstat
	subne	\irqnr, \irqnr, \irqstat
	.endm
