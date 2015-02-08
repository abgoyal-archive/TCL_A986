

#ifndef __MACH_UNCOMPRESS_H
#define __MACH_UNCOMPRESS_H

#include <linux/serial_reg.h>
#include <mach/hardware.h>

static volatile unsigned long *UART = (unsigned long *)GEMINI_UART_BASE;

static inline void putc(char c)
{
	while (!(UART[UART_LSR] & UART_LSR_THRE))
		barrier();
	UART[UART_TX] = c;
}

static inline void flush(void)
{
}

#define arch_decomp_setup()

#define arch_decomp_wdog()

#endif /* __MACH_UNCOMPRESS_H */
