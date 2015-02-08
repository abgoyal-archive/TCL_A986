

/* pull in the relevant register and map files. */

#include <mach/map.h>
#include <plat/regs-serial.h>

	.macro addruart, rx, rtmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1
		ldreq	\rx, = S3C_PA_UART
		ldrne	\rx, = S3C_VA_UART
#if CONFIG_DEBUG_S3C_UART != 0
		add	\rx, \rx, #(0x400 * CONFIG_DEBUG_S3C_UART)
#endif
	.endm

#define fifo_full fifo_full_s5pv210
#define fifo_level fifo_level_s5pv210


#include <plat/debug-macro.S>
