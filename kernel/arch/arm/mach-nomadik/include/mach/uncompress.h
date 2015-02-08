

#ifndef __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

#include <asm/setup.h>
#include <asm/io.h>
#include <mach/hardware.h>

/* we need the constants in amba/serial.h, but it refers to amba_device */
struct amba_device;
#include <linux/amba/serial.h>

#define NOMADIK_UART_DR		0x101FB000
#define NOMADIK_UART_LCRH	0x101FB02c
#define NOMADIK_UART_CR		0x101FB030
#define NOMADIK_UART_FR		0x101FB018

static void putc(const char c)
{
	/* Do nothing if the UART is not enabled. */
	if (!(readb(NOMADIK_UART_CR) & UART01x_CR_UARTEN))
		return;

	if (c == '\n')
		putc('\r');

	while (readb(NOMADIK_UART_FR) & UART01x_FR_TXFF)
		barrier();
	writeb(c, NOMADIK_UART_DR);
}

static void flush(void)
{
	if (!(readb(NOMADIK_UART_CR) & UART01x_CR_UARTEN))
		return;
	while (readb(NOMADIK_UART_FR) & UART01x_FR_BUSY)
		barrier();
}

static inline void arch_decomp_setup(void)
{
}

#define arch_decomp_wdog() /* nothing to do here */

#endif /* __ASM_ARCH_UNCOMPRESS_H */
