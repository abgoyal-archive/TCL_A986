
#include <mach/hardware.h>

	.macro	addruart, rx, tmp
	mrc	p15, 0, \rx, c1, c0
	tst	\rx, #1					@ MMU enabled?
	ldreq	\rx, =GEMINI_UART_BASE			@ physical
	ldrne	\rx, =IO_ADDRESS(GEMINI_UART_BASE)	@ virtual
	.endm

#define UART_SHIFT	2
#define FLOW_CONTROL
#include <asm/hardware/debug-8250.S>
