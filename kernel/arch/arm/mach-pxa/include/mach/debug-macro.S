

#include "hardware.h"

		.macro	addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		moveq	\rx, #0x40000000		@ physical
		movne	\rx, #io_p2v(0x40000000)	@ virtual
		orr	\rx, \rx, #0x00100000
		.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
