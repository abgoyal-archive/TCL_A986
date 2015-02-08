

		.macro	addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		moveq	\rx,      #0x10000000
		movne	\rx,      #0xf1000000	@ virtual base
		orr	\rx, \rx, #0x001F0000
		orr	\rx, \rx, #0x00001000
		.endm

#include <asm/hardware/debug-pl01x.S>
