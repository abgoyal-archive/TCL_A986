
#include <mach/hardware.h>
#include <asm/memory.h>

#include <mach/regs-board-a9m9750dev.h>

		.macro	addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1
		ldreq	\rx, =NS9XXX_CSxSTAT_PHYS(0)
		ldrne	\rx, =io_p2v(NS9XXX_CSxSTAT_PHYS(0))
		.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
