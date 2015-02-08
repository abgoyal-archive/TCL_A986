

#ifndef __ASM_PLAT_MAP_H
#define __ASM_PLAT_MAP_H __FILE__


#define S3C_ADDR_BASE	(0xF4000000)

#ifndef __ASSEMBLY__
#define S3C_ADDR(x)	((void __iomem __force *)S3C_ADDR_BASE + (x))
#else
#define S3C_ADDR(x)	(S3C_ADDR_BASE + (x))
#endif

#define S3C_VA_IRQ	S3C_ADDR(0x00000000)	/* irq controller(s) */
#define S3C_VA_SYS	S3C_ADDR(0x00100000)	/* system control */
#define S3C_VA_MEM	S3C_ADDR(0x00200000)	/* memory control */
#define S3C_VA_TIMER	S3C_ADDR(0x00300000)	/* timer block */
#define S3C_VA_WATCHDOG	S3C_ADDR(0x00400000)	/* watchdog */
#define S3C_VA_UART	S3C_ADDR(0x01000000)	/* UART */

#define S3C_ADDR_CPU(x)	S3C_ADDR(0x00500000 + (x))

#endif /* __ASM_PLAT_MAP_H */
