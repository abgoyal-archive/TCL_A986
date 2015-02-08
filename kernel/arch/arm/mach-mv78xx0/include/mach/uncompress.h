

#include <linux/serial_reg.h>
#include <mach/mv78xx0.h>

#define SERIAL_BASE	((unsigned char *)UART0_PHYS_BASE)

static void putc(const char c)
{
	unsigned char *base = SERIAL_BASE;
	int i;

	for (i = 0; i < 0x1000; i++) {
		if (base[UART_LSR << 2] & UART_LSR_THRE)
			break;
		barrier();
	}

	base[UART_TX << 2] = c;
}

static void flush(void)
{
	unsigned char *base = SERIAL_BASE;
	unsigned char mask;
	int i;

	mask = UART_LSR_TEMT | UART_LSR_THRE;

	for (i = 0; i < 0x1000; i++) {
		if ((base[UART_LSR << 2] & mask) == mask)
			break;
		barrier();
	}
}

#define arch_decomp_setup()
#define arch_decomp_wdog()
