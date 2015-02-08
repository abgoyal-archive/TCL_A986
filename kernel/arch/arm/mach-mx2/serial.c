

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <mach/hardware.h>
#include <mach/imx-uart.h>
#include "devices.h"

static struct resource uart0[] = {
	{
		.start = MX2x_UART1_BASE_ADDR,
		.end = MX2x_UART1_BASE_ADDR + 0x0B5,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MX2x_INT_UART1,
		.end = MX2x_INT_UART1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_uart_device0 = {
	.name = "imx-uart",
	.id = 0,
	.resource = uart0,
	.num_resources = ARRAY_SIZE(uart0),
};

static struct resource uart1[] = {
	{
		.start = MX2x_UART2_BASE_ADDR,
		.end = MX2x_UART2_BASE_ADDR + 0x0B5,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MX2x_INT_UART2,
		.end = MX2x_INT_UART2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_uart_device1 = {
	.name = "imx-uart",
	.id = 1,
	.resource = uart1,
	.num_resources = ARRAY_SIZE(uart1),
};

static struct resource uart2[] = {
	{
		.start = MX2x_UART3_BASE_ADDR,
		.end = MX2x_UART3_BASE_ADDR + 0x0B5,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MX2x_INT_UART3,
		.end = MX2x_INT_UART3,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_uart_device2 = {
	.name = "imx-uart",
	.id = 2,
	.resource = uart2,
	.num_resources = ARRAY_SIZE(uart2),
};

static struct resource uart3[] = {
	{
		.start = MX2x_UART4_BASE_ADDR,
		.end = MX2x_UART4_BASE_ADDR + 0x0B5,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MX2x_INT_UART4,
		.end = MX2x_INT_UART4,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_uart_device3 = {
	.name = "imx-uart",
	.id = 3,
	.resource = uart3,
	.num_resources = ARRAY_SIZE(uart3),
};

#ifdef CONFIG_MACH_MX27
static struct resource uart4[] = {
	{
		.start = MX27_UART5_BASE_ADDR,
		.end = MX27_UART5_BASE_ADDR + 0x0B5,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MX27_INT_UART5,
		.end = MX27_INT_UART5,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_uart_device4 = {
	.name = "imx-uart",
	.id = 4,
	.resource = uart4,
	.num_resources = ARRAY_SIZE(uart4),
};

static struct resource uart5[] = {
	{
		.start = MX27_UART6_BASE_ADDR,
		.end = MX27_UART6_BASE_ADDR + 0x0B5,
		.flags = IORESOURCE_MEM,
	}, {
		.start = MX27_INT_UART6,
		.end = MX27_INT_UART6,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_uart_device5 = {
	.name = "imx-uart",
	.id = 5,
	.resource = uart5,
	.num_resources = ARRAY_SIZE(uart5),
};
#endif
