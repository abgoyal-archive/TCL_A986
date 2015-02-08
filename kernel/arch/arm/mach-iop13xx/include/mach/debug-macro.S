

	.macro	addruart, rx, tmp
	mrc	p15, 0, \rx, c1, c0
	tst	\rx, #1			@ mmu enabled?
	moveq	\rx, #0xff000000	@ physical
	orreq	\rx, \rx, #0x00d80000
	movne	\rx, #0xfe000000	@ virtual
	orrne	\rx, \rx, #0x00e80000
	orr	\rx, \rx, #0x00002300
	orr	\rx, \rx, #0x00000040
	.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
