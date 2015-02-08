


#define REG(x) (*(volatile unsigned long *)(x))

#define UART1_BASE 0x100a00
#define UART2_BASE 0x100a80

#define UART_DR 0x0

#define UART_CR 0x14
#define CR_UART_EN (1<<0)

#define UART_FR 0x18
#define FR_BUSY (1<<3)
#define FR_TXFF (1<<5)

static void putc(char c)
{
	unsigned long base;

	if (REG(UART1_BASE + UART_CR) & CR_UART_EN)
		base = UART1_BASE;
	else if (REG(UART2_BASE + UART_CR) & CR_UART_EN)
		base = UART2_BASE;
	else
		return;

	while (REG(base + UART_FR) & FR_TXFF);
	REG(base + UART_DR) = c;
}

static inline void flush(void)
{
	unsigned long base;

	if (REG(UART1_BASE + UART_CR) & CR_UART_EN)
		base = UART1_BASE;
	else if (REG(UART2_BASE + UART_CR) & CR_UART_EN)
		base = UART2_BASE;
	else
		return;

	while (REG(base + UART_FR) & FR_BUSY);
}

#define arch_decomp_setup()
#define arch_decomp_wdog()
