

#include <linux/amba/serial.h>
#include <mach/spear.h>

		.macro	addruart, rx
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1					@ MMU enabled?
		moveq	\rx, #SPEAR_DBG_UART_BASE		@ Physical base
		movne	\rx, #VA_SPEAR_DBG_UART_BASE		@ Virtual base
		.endm

		.macro	senduart, rd, rx
		strb	\rd, [\rx, #UART01x_DR]			@ ASC_TX_BUFFER
		.endm

		.macro	waituart, rd, rx
1001:		ldr	\rd, [\rx, #UART01x_FR]			@ FLAG REGISTER
		tst	\rd, #UART01x_FR_TXFF			@ TX_FULL
		bne	1001b
		.endm

		.macro	busyuart, rd, rx
1002:		ldr	\rd, [\rx, #UART01x_FR]			@ FLAG REGISTER
		tst	\rd, #UART011_FR_TXFE			@ TX_EMPTY
		beq	1002b
		.endm
