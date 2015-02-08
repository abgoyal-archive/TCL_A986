
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>

#include <asm/gt64120.h>

static struct resource wrppmc_uart_resource[] __initdata = {
	{
		.start	= WRPPMC_UART16550_BASE,
		.end	= WRPPMC_UART16550_BASE + 7,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= WRPPMC_UART16550_IRQ,
		.end	= WRPPMC_UART16550_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct plat_serial8250_port wrppmc_serial8250_port[] = {
	{
		.irq		= WRPPMC_UART16550_IRQ,
		.uartclk	= WRPPMC_UART16550_CLOCK,
		.iotype		= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_SKIP_TEST,
		.mapbase	= WRPPMC_UART16550_BASE,
	},
	{},
};

static __init int wrppmc_uart_add(void)
{
	struct platform_device *pdev;
	int retval;

	pdev = platform_device_alloc("serial8250", -1);
	if (!pdev)
		return -ENOMEM;

	pdev->id = PLAT8250_DEV_PLATFORM;
	pdev->dev.platform_data = wrppmc_serial8250_port;

	retval = platform_device_add_resources(pdev, wrppmc_uart_resource,
					ARRAY_SIZE(wrppmc_uart_resource));
	if (retval)
		goto err_free_device;

	retval = platform_device_add(pdev);
	if (retval)
		goto err_free_device;

	return 0;

err_free_device:
	platform_device_put(pdev);

	return retval;
}
device_initcall(wrppmc_uart_add);
