

/* pull in the relevant register and map files. */

#include <mach/map.h>
#include <plat/regs-serial.h>

	.macro addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1
		ldreq	\rx, = S3C24XX_PA_UART
		ldrne	\rx, = S3C24XX_VA_UART
#if CONFIG_DEBUG_S3C_UART != 0
		add	\rx, \rx, #(S3C2410_UART1_OFF * CONFIG_DEBUG_S3C_UART)
#endif
	.endm


#include <plat/debug-macro.S>
