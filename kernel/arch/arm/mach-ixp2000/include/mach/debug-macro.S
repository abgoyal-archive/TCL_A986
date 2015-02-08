

		.macro  addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		moveq	\rx, #0xc0000000	@ Physical base
		movne	\rx, #0xfe000000	@ virtual base
		orrne	\rx, \rx, #0x00f00000
		orr	\rx, \rx, #0x00030000
#ifdef	__ARMEB__
		orr	\rx, \rx, #0x00000003
#endif
		.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
