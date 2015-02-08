

		.macro	addruart, rx, tmp
		mov	\rx, #0xe0000000
		orr	\rx, \rx, #0x000003f8
		.endm

		.macro	senduart,rd,rx
		strb	\rd, [\rx]
		.endm

		.macro waituart,rd,rx
		.endm

		.macro	busyuart,rd,rx
		mov	\rd, #0
1001:		add	\rd, \rd, #1
		teq	\rd, #0x10000
		bne	1001b
		.endm

