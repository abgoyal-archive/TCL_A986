

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/map.h>

#include <plat/adc.h>
#include <plat/devs.h>
#include <plat/cpu.h>

static struct resource s3c_adc_resource[] = {
	[0] = {
		.start = SAMSUNG_PA_ADC,
		.end   = SAMSUNG_PA_ADC + SZ_256 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_TC,
		.end   = IRQ_TC,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_ADC,
		.end   = IRQ_ADC,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device s3c_device_adc = {
	.name		= "samsung-adc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(s3c_adc_resource),
	.resource	= s3c_adc_resource,
};
