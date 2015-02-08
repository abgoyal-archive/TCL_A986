
#include <mach/hardware.h>
#include <mach/irqs.h>

	.macro  disable_fiq
	.endm

	.macro  get_irqnr_preamble, base, tmp
	ldr     \base, =INTFLGA
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm

	.macro  get_irqnr_and_base, irqnr, irqstat, base, tmp
	ldr     \irqnr, [\base]
	cmp	\irqnr, #0
	beq	1000f
	/* intevt to irq number */
	lsr	\irqnr, \irqnr, #0x5
	subs	\irqnr, \irqnr, #16

1000:
	.endm
