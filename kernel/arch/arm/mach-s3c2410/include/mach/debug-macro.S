

#include <mach/map.h>
#include <mach/regs-gpio.h>
#include <plat/regs-serial.h>

#define S3C2410_UART1_OFF (0x4000)
#define SHIFT_2440TXF (14-9)

	.macro addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1
		ldreq	\rx, = S3C24XX_PA_UART
		ldrne	\rx, = S3C24XX_VA_UART
#if CONFIG_DEBUG_S3C_UART != 0
		add	\rx, \rx, #(S3C2410_UART1_OFF * CONFIG_DEBUG_S3C_UART)
#endif
	.endm

	.macro fifo_full_s3c24xx rd, rx
		@ check for arm920 vs arm926. currently assume all arm926
		@ devices have an 64 byte FIFO identical to the s3c2440
		mrc	p15, 0, \rd, c0, c0
		and	\rd, \rd, #0xff0
		teq	\rd, #0x260
		beq	1004f
		mrc	p15, 0, \rd, c1, c0
		tst	\rd, #1
		addeq	\rd, \rx, #(S3C24XX_PA_GPIO - S3C24XX_PA_UART)
		addne	\rd, \rx, #(S3C24XX_VA_GPIO - S3C24XX_VA_UART)
		bic	\rd, \rd, #0xff000
		ldr	\rd, [ \rd, # S3C2410_GSTATUS1 - S3C2410_GPIOREG(0) ]
		and	\rd, \rd, #0x00ff0000
		teq	\rd, #0x00440000		@ is it 2440?
1004:
		ldr	\rd, [ \rx, # S3C2410_UFSTAT ]
		moveq	\rd, \rd, lsr #SHIFT_2440TXF
		tst	\rd, #S3C2410_UFSTAT_TXFULL
	.endm

	.macro  fifo_full_s3c2410 rd, rx
		ldr	\rd, [ \rx, # S3C2410_UFSTAT ]
		tst	\rd, #S3C2410_UFSTAT_TXFULL
	.endm

/* fifo level reading */

	.macro fifo_level_s3c24xx rd, rx
		@ check for arm920 vs arm926. currently assume all arm926
		@ devices have an 64 byte FIFO identical to the s3c2440
		mrc	p15, 0, \rd, c0, c0
		and	\rd, \rd, #0xff0
		teq	\rd, #0x260
		beq	10000f
		mrc	p15, 0, \rd, c1, c0
		tst	\rd, #1
		addeq	\rd, \rx, #(S3C24XX_PA_GPIO - S3C24XX_PA_UART)
		addne	\rd, \rx, #(S3C24XX_VA_GPIO - S3C24XX_VA_UART)
		bic	\rd, \rd, #0xff000
		ldr	\rd, [ \rd, # S3C2410_GSTATUS1 - S3C2410_GPIOREG(0) ]
		and	\rd, \rd, #0x00ff0000
		teq	\rd, #0x00440000		@ is it 2440?

10000:
		ldr	\rd, [ \rx, # S3C2410_UFSTAT ]
		andne	\rd, \rd, #S3C2410_UFSTAT_TXMASK
		andeq	\rd, \rd, #S3C2440_UFSTAT_TXMASK
	.endm

	.macro fifo_level_s3c2410 rd, rx
		ldr	\rd, [ \rx, # S3C2410_UFSTAT ]
		and	\rd, \rd, #S3C2410_UFSTAT_TXMASK
	.endm


#if defined(CONFIG_CPU_LLSERIAL_S3C2410_ONLY)
#define fifo_full  fifo_full_s3c2410
#define fifo_level fifo_level_s3c2410
#elif !defined(CONFIG_CPU_LLSERIAL_S3C2440_ONLY)
#define fifo_full  fifo_full_s3c24xx
#define fifo_level fifo_level_s3c24xx
#endif

/* include the reset of the code which will do the work */

#include <plat/debug-macro.S>
