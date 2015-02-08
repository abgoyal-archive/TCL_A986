

#include <mach/addr-map.h>

		.macro	addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1				@ MMU enabled?
		ldreq	\rx, =APB_PHYS_BASE		@ physical
		ldrne	\rx, =APB_VIRT_BASE		@ virtual
		orr	\rx, \rx, #0x00017000
		.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
