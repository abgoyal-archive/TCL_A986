

		.macro	addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ mmu enabled?
		moveq	\rx, #0xff000000	@ physical
		movne	\rx, #0xfe000000	@ virtual
		orr	\rx, \rx, #0x00ff0000
		orr	\rx, \rx, #0x0000f700
		.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
