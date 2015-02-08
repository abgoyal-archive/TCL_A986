

#ifndef __ASM_PLAT_S3C24XX_MAP_H
#define __ASM_PLAT_S3C24XX_MAP_H

#define S3C24XX_VA_IRQ	   S3C_VA_IRQ
#define S3C2410_PA_IRQ	   (0x4A000000)
#define S3C24XX_SZ_IRQ	   SZ_1M

/* memory controller registers */
#define S3C24XX_VA_MEMCTRL S3C_VA_MEM
#define S3C2410_PA_MEMCTRL (0x48000000)
#define S3C24XX_SZ_MEMCTRL SZ_1M

/* UARTs */
#define S3C24XX_VA_UART	   S3C_VA_UART
#define S3C2410_PA_UART	   (0x50000000)
#define S3C24XX_SZ_UART	   SZ_1M
#define S3C_UART_OFFSET	   (0x4000)

#define S3C_VA_UARTx(uart) (S3C_VA_UART + ((uart * S3C_UART_OFFSET)))

/* Timers */
#define S3C24XX_VA_TIMER   S3C_VA_TIMER
#define S3C2410_PA_TIMER   (0x51000000)
#define S3C24XX_SZ_TIMER   SZ_1M

/* Clock and Power management */
#define S3C24XX_VA_CLKPWR  S3C_VA_SYS
#define S3C24XX_SZ_CLKPWR  SZ_1M

/* USB Device port */
#define S3C2410_PA_USBDEV  (0x52000000)
#define S3C24XX_SZ_USBDEV  SZ_1M

/* Watchdog */
#define S3C24XX_VA_WATCHDOG S3C_VA_WATCHDOG
#define S3C2410_PA_WATCHDOG (0x53000000)
#define S3C24XX_SZ_WATCHDOG SZ_1M

/* Standard size definitions for peripheral blocks. */

#define S3C24XX_SZ_IIS		SZ_1M
#define S3C24XX_SZ_ADC		SZ_1M
#define S3C24XX_SZ_SPI		SZ_1M
#define S3C24XX_SZ_SDI		SZ_1M
#define S3C24XX_SZ_NAND		SZ_1M

/* GPIO ports */


#define S3C2410_PA_GPIO	   (0x56000000)
#define S3C24XX_VA_GPIO	   ((S3C24XX_PA_GPIO - S3C24XX_PA_UART) + S3C24XX_VA_UART)
#define S3C24XX_SZ_GPIO	   SZ_1M



#define S3C24XX_VA_ISA_WORD  S3C2410_ADDR(0x02000000)
#define S3C24XX_VA_ISA_BYTE  S3C2410_ADDR(0x03000000)

/* deal with the registers that move under the 2412/2413 */

#if defined(CONFIG_CPU_S3C2412) || defined(CONFIG_CPU_S3C2413)
#ifndef __ASSEMBLY__
extern void __iomem *s3c24xx_va_gpio2;
#endif
#ifdef CONFIG_CPU_S3C2412_ONLY
#define S3C24XX_VA_GPIO2 (S3C24XX_VA_GPIO + 0x10)
#else
#define S3C24XX_VA_GPIO2 s3c24xx_va_gpio2
#endif
#else
#define s3c24xx_va_gpio2 S3C24XX_VA_GPIO
#define S3C24XX_VA_GPIO2 S3C24XX_VA_GPIO
#endif

#endif /* __ASM_PLAT_S3C24XX_MAP_H */
