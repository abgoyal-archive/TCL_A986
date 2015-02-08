

#include <mach/mv78xx0.h>

	.macro	addruart, rx, tmp
	mrc	p15, 0, \rx, c1, c0
	tst	\rx, #1					@ MMU enabled?
	ldreq	\rx, =MV78XX0_REGS_PHYS_BASE
	ldrne	\rx, =MV78XX0_REGS_VIRT_BASE
	orr	\rx, \rx, #0x00012000
	.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
