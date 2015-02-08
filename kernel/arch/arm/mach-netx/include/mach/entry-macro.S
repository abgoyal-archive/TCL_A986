
#include <mach/hardware.h>

		.macro  disable_fiq
		.endm

		.macro  get_irqnr_preamble, base, tmp
		ldr	\base, =io_p2v(0x001ff000)
		.endm

		.macro  arch_ret_to_user, tmp1, tmp2
		.endm

		.macro  get_irqnr_and_base, irqnr, irqstat, base, tmp
		ldr	\irqstat, [\base, #0]
		clz	\irqnr, \irqstat
		rsb     \irqnr, \irqnr, #31
		cmp	\irqstat, #0
		.endm

