

#include <linux/init.h>
#include <linux/tty.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>

#include <asm/serial.h>
#include <asm/mach-rc32434/rb.h>

extern unsigned int idt_cpu_freq;

static struct uart_port rb532_uart = {
	.flags = UPF_BOOT_AUTOCONF,
	.line = 0,
	.irq = UART0_IRQ,
	.iotype = UPIO_MEM,
	.membase = (char *)KSEG1ADDR(REGBASE + UART0BASE),
	.regshift = 2
};

int __init setup_serial_port(void)
{
	rb532_uart.uartclk = idt_cpu_freq;

	return early_serial_setup(&rb532_uart);
}
arch_initcall(setup_serial_port);
