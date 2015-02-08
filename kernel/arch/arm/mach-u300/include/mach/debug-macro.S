
#include <mach/hardware.h>

	.macro	addruart, rx, tmp
	/* If we move the address using MMU, use this. */
	mrc	p15, 0, \rx, c1, c0
	tst	\rx, #1			@ MMU enabled?
	ldreq	\rx,	  = U300_SLOW_PER_PHYS_BASE @ MMU off, physical address
	ldrne	\rx,	  = U300_SLOW_PER_VIRT_BASE @ MMU on, virtual address
	orr	\rx, \rx, #0x00003000
	.endm

#include <asm/hardware/debug-pl01x.S>
