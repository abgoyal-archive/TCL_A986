

#ifndef __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

#include "hardware.h"

#define UART(x)         (*(volatile unsigned long *)(serial_port + (x)))

static void putc(int c)
{
	unsigned long serial_port;
        do {
		serial_port = _UART3_BASE;
		if (UART(UART_CR) & UART_CR_EN) break;
		serial_port = _UART1_BASE;
		if (UART(UART_CR) & UART_CR_EN) break;
		serial_port = _UART2_BASE;
		if (UART(UART_CR) & UART_CR_EN) break;
		return;
	} while (0);

	/* wait for space in the UART's transmitter */
	while ((UART(UART_SR) & UART_SR_TxFF))
		barrier();

	/* send the character out. */
	UART(UART_DR) = c;
}

static inline void flush(void)
{
}

#define arch_decomp_setup()
#define arch_decomp_wdog()

#endif /* __ASM_ARCH_UNCOMPRESS_H */
