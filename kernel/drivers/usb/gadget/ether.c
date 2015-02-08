

/* #define VERBOSE_DEBUG */

#include <linux/kernel.h>
#include <linux/utsname.h>


#if defined USB_ETH_RNDIS
#  undef USB_ETH_RNDIS
#endif
#ifdef CONFIG_USB_ETH_RNDIS
#  define USB_ETH_RNDIS y
#endif

#include "u_ether.h"



#define DRIVER_DESC		"Ethernet Gadget"
#define DRIVER_VERSION		"Memorial Day 2008"

#ifdef USB_ETH_RNDIS
#define PREFIX			"RNDIS/"
#else
#define PREFIX			""
#endif


static inline bool has_rndis(void)
{
#ifdef	USB_ETH_RNDIS
	return true;
#else
	return false;
#endif
}

/*-------------------------------------------------------------------------*/

#include "composite.c"
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"

#include "f_ecm.c"
#include "f_subset.c"
#ifdef	USB_ETH_RNDIS
#include "f_rndis.c"
#include "rndis.c"
#endif
#include "f_eem.c"
#include "u_ether.c"

/*-------------------------------------------------------------------------*/


#define CDC_VENDOR_NUM		0x0525	/* NetChip */
#define CDC_PRODUCT_NUM		0xa4a1	/* Linux-USB Ethernet Gadget */

#define	SIMPLE_VENDOR_NUM	0x049f
#define	SIMPLE_PRODUCT_NUM	0x505a

#define RNDIS_VENDOR_NUM	0x0525	/* NetChip */
#define RNDIS_PRODUCT_NUM	0xa4a2	/* Ethernet/RNDIS Gadget */

/* For EEM gadgets */
#define EEM_VENDOR_NUM		0x1d6b	/* Linux Foundation */
#define EEM_PRODUCT_NUM		0x0102	/* EEM Gadget */

/*-------------------------------------------------------------------------*/

static struct usb_device_descriptor device_desc = {
	.bLength =		sizeof device_desc,
	.bDescriptorType =	USB_DT_DEVICE,

	.bcdUSB =		cpu_to_le16 (0x0200),

	.bDeviceClass =		USB_CLASS_COMM,
	.bDeviceSubClass =	0,
	.bDeviceProtocol =	0,
	/* .bMaxPacketSize0 = f(hardware) */

	/* Vendor and product id defaults change according to what configs
	 * we support.  (As does bNumConfigurations.)  These values can
	 * also be overridden by module parameters.
	 */
	.idVendor =		cpu_to_le16 (CDC_VENDOR_NUM),
	.idProduct =		cpu_to_le16 (CDC_PRODUCT_NUM),
	/* .bcdDevice = f(hardware) */
	/* .iManufacturer = DYNAMIC */
	/* .iProduct = DYNAMIC */
	/* NO SERIAL NUMBER */
	.bNumConfigurations =	1,
};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,

	/* REVISIT SRP-only hardware is possible, although
	 * it would not be called "OTG" ...
	 */
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};


/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1

static char manufacturer[50];

static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer,
	[STRING_PRODUCT_IDX].s = PREFIX DRIVER_DESC,
	{  } /* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static u8 hostaddr[ETH_ALEN];

/*-------------------------------------------------------------------------*/

static int __init rndis_do_config(struct usb_configuration *c)
{
	/* FIXME alloc iConfiguration string, set it in c->strings */

	if (gadget_is_otg(c->cdev->gadget)) {
		c->descriptors = otg_desc;
		c->bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	return rndis_bind_config(c, hostaddr);
}

static struct usb_configuration rndis_config_driver = {
	.label			= "RNDIS",
	.bind			= rndis_do_config,
	.bConfigurationValue	= 2,
	/* .iConfiguration = DYNAMIC */
	.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,
};

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_USB_ETH_EEM
static int use_eem = 1;
#else
static int use_eem;
#endif
module_param(use_eem, bool, 0);
MODULE_PARM_DESC(use_eem, "use CDC EEM mode");

static int __init eth_do_config(struct usb_configuration *c)
{
	/* FIXME alloc iConfiguration string, set it in c->strings */

	if (gadget_is_otg(c->cdev->gadget)) {
		c->descriptors = otg_desc;
		c->bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	if (use_eem)
		return eem_bind_config(c);
	else if (can_support_ecm(c->cdev->gadget))
		return ecm_bind_config(c, hostaddr);
	else
		return geth_bind_config(c, hostaddr);
}

static struct usb_configuration eth_config_driver = {
	/* .label = f(hardware) */
	.bind			= eth_do_config,
	.bConfigurationValue	= 1,
	/* .iConfiguration = DYNAMIC */
	.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,
};

/*-------------------------------------------------------------------------*/

static int __init eth_bind(struct usb_composite_dev *cdev)
{
	int			gcnum;
	struct usb_gadget	*gadget = cdev->gadget;
	int			status;

	/* set up network link layer */
	status = gether_setup(cdev->gadget, hostaddr);
	if (status < 0)
		return status;

	/* set up main config label and device descriptor */
	if (use_eem) {
		/* EEM */
		eth_config_driver.label = "CDC Ethernet (EEM)";
		device_desc.idVendor = cpu_to_le16(EEM_VENDOR_NUM);
		device_desc.idProduct = cpu_to_le16(EEM_PRODUCT_NUM);
	} else if (can_support_ecm(cdev->gadget)) {
		/* ECM */
		eth_config_driver.label = "CDC Ethernet (ECM)";
	} else {
		/* CDC Subset */
		eth_config_driver.label = "CDC Subset/SAFE";

		device_desc.idVendor = cpu_to_le16(SIMPLE_VENDOR_NUM);
		device_desc.idProduct = cpu_to_le16(SIMPLE_PRODUCT_NUM);
		if (!has_rndis())
			device_desc.bDeviceClass = USB_CLASS_VENDOR_SPEC;
	}

	if (has_rndis()) {
		/* RNDIS plus ECM-or-Subset */
		device_desc.idVendor = cpu_to_le16(RNDIS_VENDOR_NUM);
		device_desc.idProduct = cpu_to_le16(RNDIS_PRODUCT_NUM);
		device_desc.bNumConfigurations = 2;
	}

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0300 | gcnum);
	else {
		/* We assume that can_support_ecm() tells the truth;
		 * but if the controller isn't recognized at all then
		 * that assumption is a bit more likely to be wrong.
		 */
		dev_warn(&gadget->dev,
				"controller '%s' not recognized; trying %s\n",
				gadget->name,
				eth_config_driver.label);
		device_desc.bcdDevice =
			cpu_to_le16(0x0300 | 0x0099);
	}


	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */

	/* device descriptor strings: manufacturer, product */
	snprintf(manufacturer, sizeof manufacturer, "%s %s with %s",
		init_utsname()->sysname, init_utsname()->release,
		gadget->name);
	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;
	strings_dev[STRING_MANUFACTURER_IDX].id = status;
	device_desc.iManufacturer = status;

	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;
	strings_dev[STRING_PRODUCT_IDX].id = status;
	device_desc.iProduct = status;

	/* register our configuration(s); RNDIS first, if it's used */
	if (has_rndis()) {
		status = usb_add_config(cdev, &rndis_config_driver);
		if (status < 0)
			goto fail;
	}

	status = usb_add_config(cdev, &eth_config_driver);
	if (status < 0)
		goto fail;

	dev_info(&gadget->dev, "%s, version: " DRIVER_VERSION "\n",
			DRIVER_DESC);

	return 0;

fail:
	gether_cleanup();
	return status;
}

static int __exit eth_unbind(struct usb_composite_dev *cdev)
{
	gether_cleanup();
	return 0;
}

static struct usb_composite_driver eth_driver = {
	.name		= "g_ether",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= eth_bind,
	.unbind		= __exit_p(eth_unbind),
};

MODULE_DESCRIPTION(PREFIX DRIVER_DESC);
MODULE_AUTHOR("David Brownell, Benedikt Spanger");
MODULE_LICENSE("GPL");

static int __init init(void)
{
	return usb_composite_register(&eth_driver);
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&eth_driver);
}
module_exit(cleanup);
