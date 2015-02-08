

#include <linux/serial_reg.h>

#define SERIAL_BASE	((unsigned char *)0xf0000be0)

static inline void putc(int c)
{
	unsigned char v, *base = SERIAL_BASE;

	do {
		v = base[UART_LSR << 2];
		barrier();
	} while (!(v & UART_LSR_THRE));

	base[UART_TX << 2] = c;
}

static inline void flush(void)
{
	unsigned char v, *base = SERIAL_BASE;

	do {
		v = base[UART_LSR << 2];
		barrier();
	} while ((v & (UART_LSR_TEMT|UART_LSR_THRE)) !=
		 (UART_LSR_TEMT|UART_LSR_THRE));
}

#define arch_decomp_setup()
#define arch_decomp_wdog()
