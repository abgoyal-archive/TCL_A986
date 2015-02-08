

#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>

#include <asm/mipsregs.h>

#include <msp_regs.h>
#include <msp_int.h>
#include <msp_prom.h>

#if defined(CONFIG_USB_EHCI_HCD)
static struct resource msp_usbhost_resources [] = {
	[0] = {
		.start	= MSP_USB_BASE_START,
		.end	= MSP_USB_BASE_END,
		.flags 	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSP_INT_USB,
		.end	= MSP_INT_USB,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 msp_usbhost_dma_mask = DMA_BIT_MASK(32);

static struct platform_device msp_usbhost_device = {
	.name	= "pmcmsp-ehci",
	.id	= 0,
	.dev	= {
		.dma_mask = &msp_usbhost_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.num_resources 	= ARRAY_SIZE(msp_usbhost_resources),
	.resource	= msp_usbhost_resources,
};
#endif /* CONFIG_USB_EHCI_HCD */

#if defined(CONFIG_USB_GADGET)
static struct resource msp_usbdev_resources [] = {
	[0] = {
		.start	= MSP_USB_BASE,
		.end	= MSP_USB_BASE_END,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSP_INT_USB,
		.end	= MSP_INT_USB,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 msp_usbdev_dma_mask = DMA_BIT_MASK(32);

static struct platform_device msp_usbdev_device = {
	.name	= "msp71xx_udc",
	.id	= 0,
	.dev	= {
		.dma_mask = &msp_usbdev_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.num_resources	= ARRAY_SIZE(msp_usbdev_resources),
	.resource	= msp_usbdev_resources,
};
#endif /* CONFIG_USB_GADGET */

#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_GADGET)
static struct platform_device *msp_devs[1];
#endif


static int __init msp_usb_setup(void)
{
#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_GADGET)
	char *strp;
	char envstr[32];
	unsigned int val = 0;
	int result = 0;

	/*
	 * construct environment name usbmode
	 * set usbmode <host/device> as pmon environment var
	 */
	snprintf((char *)&envstr[0], sizeof(envstr), "usbmode");

#if defined(CONFIG_USB_EHCI_HCD)
	/* default to host mode */
	val = 1;
#endif

	/* get environment string */
	strp = prom_getenv((char *)&envstr[0]);
	if (strp) {
		if (!strcmp(strp, "device"))
			val = 0;
	}

	if (val) {
#if defined(CONFIG_USB_EHCI_HCD)
		/* get host mode device */
		msp_devs[0] = &msp_usbhost_device;
		ppfinit("platform add USB HOST done %s.\n",
			    msp_devs[0]->name);

		result = platform_add_devices(msp_devs, ARRAY_SIZE(msp_devs));
#endif /* CONFIG_USB_EHCI_HCD */
	}
#if defined(CONFIG_USB_GADGET)
	else {
		/* get device mode structure */
		msp_devs[0] = &msp_usbdev_device;
		ppfinit("platform add USB DEVICE done %s.\n",
			    msp_devs[0]->name);

		result = platform_add_devices(msp_devs, ARRAY_SIZE(msp_devs));
	}
#endif /* CONFIG_USB_GADGET */
#endif /* CONFIG_USB_EHCI_HCD || CONFIG_USB_GADGET */

	return result;
}

subsys_initcall(msp_usb_setup);
