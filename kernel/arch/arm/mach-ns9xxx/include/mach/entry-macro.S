
#include <mach/hardware.h>
#include <mach/regs-sys-common.h>

		.macro	get_irqnr_preamble, base, tmp
		ldr	\base, =SYS_ISRADDR
		.endm

		.macro	arch_ret_to_user, tmp1, tmp2
		.endm

		.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
		ldr	\irqstat, [\base, #(SYS_ISA - SYS_ISRADDR)]
		cmp	\irqstat, #0
		ldrne	\irqnr, [\base]
		.endm

		.macro	disable_fiq
		.endm
