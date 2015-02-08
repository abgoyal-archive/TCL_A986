


		.macro	disable_fiq
		.endm

		.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp

	        mov	\base, #0xf0000000	@ vm address of IRQ controller
		ldr	\irqnr, [\base, #0x30]	@ HW_ICOLL_STAT
		cmp	\irqnr, #0x3f
		movne	\irqstat, #0		@ Ack this IRQ
		strne	\irqstat, [\base, #0x00]@ HW_ICOLL_VECTOR
		moveqs	\irqnr, #0		@ Zero flag set for no IRQ

		.endm

                .macro  get_irqnr_preamble, base, tmp
                .endm

                .macro  arch_ret_to_user, tmp1, tmp2
                .endm
