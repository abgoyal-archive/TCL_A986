

		.macro	addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		moveq	\rx, #0x16000000	@ physical base address
		movne	\rx, #0xf0000000	@ virtual base
		addne	\rx, \rx, #0x16000000 >> 4
		.endm

#include <asm/hardware/debug-pl01x.S>
