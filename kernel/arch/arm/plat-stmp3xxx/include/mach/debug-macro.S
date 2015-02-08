


		.macro	addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		moveq	\rx, #0x80000000	@ physical base address
		addeq	\rx, \rx, #0x00070000
		movne	\rx, #0xf0000000	@ virtual base
		addne	\rx, \rx, #0x00070000
		.endm

		.macro	senduart,rd,rx
		strb	\rd, [\rx, #0]		@ data register at 0
		.endm

		.macro	waituart,rd,rx
1001:		ldr	\rd, [\rx, #0x18]	@ UARTFLG
		tst	\rd, #1 << 5		@ UARTFLGUTXFF - 1 when full
		bne	1001b
		.endm

		.macro	busyuart,rd,rx
1001:		ldr	\rd, [\rx, #0x18]	@ UARTFLG
		tst	\rd, #1 << 3		@ UARTFLGUBUSY - 1 when busy
		bne	1001b
		.endm
