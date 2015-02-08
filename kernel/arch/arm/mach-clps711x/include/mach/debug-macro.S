

#include <mach/hardware.h>
#include <asm/hardware/clps7111.h>

		.macro	addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		moveq	\rx, #CLPS7111_PHYS_BASE
		movne	\rx, #CLPS7111_VIRT_BASE
#ifndef CONFIG_DEBUG_CLPS711X_UART2
		add	\rx, \rx, #0x0000	@ UART1
#else
		add	\rx, \rx, #0x1000	@ UART2
#endif
		.endm

		.macro	senduart,rd,rx
		str	\rd, [\rx, #0x0480]	@ UARTDR
		.endm

		.macro	waituart,rd,rx
1001:		ldr	\rd, [\rx, #0x0140]	@ SYSFLGx
		tst	\rd, #1 << 11		@ UBUSYx
		bne	1001b
		.endm

		.macro	busyuart,rd,rx
		tst	\rx, #0x1000		@ UART2 does not have CTS here
		bne	1002f
1001:		ldr	\rd, [\rx, #0x0140]	@ SYSFLGx
		tst	\rd, #1 << 8		@ CTS
		bne	1001b
1002:
		.endm

