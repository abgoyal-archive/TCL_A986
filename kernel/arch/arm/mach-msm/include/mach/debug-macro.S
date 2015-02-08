



#include <mach/hardware.h>
#include <mach/msm_iomap.h>

#ifdef CONFIG_MSM_DEBUG_UART
	.macro	addruart, rx, tmp
	@ see if the MMU is enabled and select appropriate base address
	mrc	p15, 0, \rx, c1, c0
	tst	\rx, #1
	ldreq	\rx, =MSM_DEBUG_UART_PHYS
	ldrne	\rx, =MSM_DEBUG_UART_BASE
	.endm

	.macro	senduart,rd,rx
	teq	\rx, #0
	strne	\rd, [\rx, #0x0C]
	.endm

	.macro	waituart,rd,rx
	@ wait for TX_READY
1001:	ldr	\rd, [\rx, #0x08]
	tst	\rd, #0x04
	beq	1001b
	.endm
#else
	.macro	addruart, rx, tmp
	.endm

	.macro	senduart,rd,rx
	.endm

	.macro	waituart,rd,rx
	.endm
#endif

	.macro	busyuart,rd,rx
	.endm
