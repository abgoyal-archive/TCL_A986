

#ifndef __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

/* Defines for UART registers */

#include <mach/regs-serial.h>
#include <mach/map.h>
#include <linux/serial_reg.h>

#define arch_decomp_wdog()

#define TX_DONE	(UART_LSR_TEMT | UART_LSR_THRE)
static u32 * uart_base = (u32 *)UART0_PA;

static void putc(int ch)
{
	/* Check THRE and TEMT bits before we transmit the character.
	 */
	while ((uart_base[UART_LSR] & TX_DONE) != TX_DONE)
		barrier();

	*uart_base = ch;
}

static inline void flush(void)
{
}

static void arch_decomp_setup(void)
{
}

#endif/* __ASM_NUC93X_UNCOMPRESS_H */
