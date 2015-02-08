
#include <mach/hardware.h>

		.equ	irq_base_addr,	IO_BASE_2

		.macro  disable_fiq
		.endm

		.macro  get_irqnr_preamble, base, tmp
		.endm

		.macro  arch_ret_to_user, tmp1, tmp2
		.endm

		.macro  get_irqnr_and_base, irqnr, irqstat, base, tmp
		mov     \irqstat, #irq_base_addr		@ Virt addr IRQ regs
		add	\irqstat, \irqstat, #0x00001000		@ Status reg
		ldr     \irqstat, [\irqstat, #0]		@ get interrupts
		mov     \irqnr, #0
1001:		tst     \irqstat, #1
		addeq   \irqnr, \irqnr, #1
		moveq   \irqstat, \irqstat, lsr #1
		tsteq   \irqnr, #32
		beq     1001b
		teq     \irqnr, #32
		.endm

