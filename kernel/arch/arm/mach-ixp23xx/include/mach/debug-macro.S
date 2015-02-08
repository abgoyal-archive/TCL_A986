
#include <mach/ixp23xx.h>

		.macro	addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1				@ mmu enabled?
		ldreq	\rx, =IXP23XX_PERIPHERAL_PHYS 	@ physical
		ldrne	\rx, =IXP23XX_PERIPHERAL_VIRT	@ virtual
#ifdef __ARMEB__
		orr	\rx, \rx, #0x00000003
#endif
		.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
