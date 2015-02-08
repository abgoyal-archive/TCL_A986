

		.macro	addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		mov	\rx, #0x00090000
		addeq	\rx, \rx, #0x40000000
		addne	\rx, \rx, #0xf4000000
		.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
