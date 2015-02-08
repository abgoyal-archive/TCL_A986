

/* pull in the relevant register and map files. */

#include <mach/map.h>
#include <plat/regs-serial.h>

	/* note, for the boot process to work we have to keep the UART
	 * virtual address aligned to an 1MiB boundary for the L1
	 * mapping the head code makes. We keep the UART virtual address
	 * aligned and add in the offset when we load the value here.
	 */

	.macro addruart, rx, tmp
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
