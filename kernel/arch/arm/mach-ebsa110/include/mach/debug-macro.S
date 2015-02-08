

		.macro	addruart, rx, tmp
		mov	\rx, #0xf0000000
		orr	\rx, \rx, #0x00000be0
		.endm

#define UART_SHIFT	2
#define FLOW_CONTROL
#include <asm/hardware/debug-8250.S>
