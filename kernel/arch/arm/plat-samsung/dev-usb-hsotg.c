

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <mach/irqs.h>
#include <mach/map.h>

#include <plat/devs.h>

static struct resource s3c_usb_hsotg_resources[] = {
	[0] = {
		.start	= S3C_PA_USB_HSOTG,
		.end	= S3C_PA_USB_HSOTG + 0x10000 - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_OTG,
		.end	= IRQ_OTG,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 s3c_hsotg_dmamask = DMA_BIT_MASK(32);

struct platform_device s3c_device_usb_hsotg = {
	.name		= "s3c-hsotg",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(s3c_usb_hsotg_resources),
	.resource	= s3c_usb_hsotg_resources,
	.dev		= {
		.dma_mask		= &s3c_hsotg_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
};
