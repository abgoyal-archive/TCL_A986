
#include <mach/ep93xx-regs.h>

		.macro	addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1				@ MMU enabled?
		ldreq	\rx, =EP93XX_APB_PHYS_BASE	@ Physical base
		ldrne	\rx, =EP93XX_APB_VIRT_BASE	@ virtual base
		orr	\rx, \rx, #0x000c0000
		.endm

#include <asm/hardware/debug-pl01x.S>
