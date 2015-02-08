

		.macro	addruart, rx, tmp
		mov	\rx, #0xfe000000	@ physical as well as virtual
		orr	\rx, \rx, #0x00800000	@ location of the UART
		.endm

#define UART_SHIFT	0
#include <asm/hardware/debug-8250.S>
