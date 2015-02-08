

		.equ	io_virt, IO_BASE
		.equ	io_phys, IO_START

		.macro	addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		moveq	\rx, #io_phys		@ physical base address
		movne	\rx, #io_virt		@ virtual address
		add	\rx, \rx, #0x00044000	@ UART1
@		add	\rx, \rx, #0x00045000	@ UART2
		.endm

		.macro	senduart,rd,rx
		str	\rd, [\rx, #0x0]	@ UARTDR
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
