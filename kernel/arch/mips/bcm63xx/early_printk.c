

#include <linux/init.h>
#include <bcm63xx_io.h>
#include <bcm63xx_regs.h>

static void __init wait_xfered(void)
{
	unsigned int val;

	/* wait for any previous char to be transmitted */
	do {
		val = bcm_uart0_readl(UART_IR_REG);
		if (val & UART_IR_STAT(UART_IR_TXEMPTY))
			break;
	} while (1);
}

void __init prom_putchar(char c)
{
	wait_xfered();
	bcm_uart0_writel(c, UART_FIFO_REG);
	wait_xfered();
}
