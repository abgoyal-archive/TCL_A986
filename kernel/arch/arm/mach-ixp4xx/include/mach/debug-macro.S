

                .macro  addruart, rx, tmp
                mrc     p15, 0, \rx, c1, c0
                tst     \rx, #1                 @ MMU enabled?
                moveq   \rx, #0xc8000000
                movne   \rx, #0xff000000
		orrne	\rx, \rx, #0x00b00000
                add     \rx,\rx,#3              @ Uart regs are at off set of 3 if
						@ byte writes used - Big Endian.
                .endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
