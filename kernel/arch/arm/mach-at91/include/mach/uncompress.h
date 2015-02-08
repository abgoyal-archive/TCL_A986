

#ifndef __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

#include <linux/io.h>
#include <linux/atmel_serial.h>

#if defined(CONFIG_AT91_EARLY_DBGU)
#define UART_OFFSET (AT91_DBGU + AT91_BASE_SYS)
#elif defined(CONFIG_AT91_EARLY_USART0)
#define UART_OFFSET AT91_USART0
#elif defined(CONFIG_AT91_EARLY_USART1)
#define UART_OFFSET AT91_USART1
#elif defined(CONFIG_AT91_EARLY_USART2)
#define UART_OFFSET AT91_USART2
#elif defined(CONFIG_AT91_EARLY_USART3)
#define UART_OFFSET AT91_USART3
#elif defined(CONFIG_AT91_EARLY_USART4)
#define UART_OFFSET AT91_USART4
#elif defined(CONFIG_AT91_EARLY_USART5)
#define UART_OFFSET AT91_USART5
#endif

static void putc(int c)
{
#ifdef UART_OFFSET
	void __iomem *sys = (void __iomem *) UART_OFFSET;	/* physical address */

	while (!(__raw_readl(sys + ATMEL_US_CSR) & ATMEL_US_TXRDY))
		barrier();
	__raw_writel(c, sys + ATMEL_US_THR);
#endif
}

static inline void flush(void)
{
#ifdef UART_OFFSET
	void __iomem *sys = (void __iomem *) UART_OFFSET;	/* physical address */

	/* wait for transmission to complete */
	while (!(__raw_readl(sys + ATMEL_US_CSR) & ATMEL_US_TXEMPTY))
		barrier();
#endif
}

#define arch_decomp_setup()

#define arch_decomp_wdog()

#endif
