




#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/usb/ch9.h>


/*-------------------------------------------------------------------------*/

#define DRIVER_DESC		"Mass Storage Gadget"
#define DRIVER_VERSION		"2009/09/11"

/*-------------------------------------------------------------------------*/


#include "composite.c"
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "f_mass_storage.c"

/*-------------------------------------------------------------------------*/

static struct usb_device_descriptor msg_device_desc = {
	.bLength =		sizeof msg_device_desc,
	.bDescriptorType =	USB_DT_DEVICE,

	.bcdUSB =		cpu_to_le16(0x0200),
	.bDeviceClass =		USB_CLASS_PER_INTERFACE,

	/* Vendor and product id can be overridden by module parameters.  */
	.idVendor =		cpu_to_le16(FSG_VENDOR_ID),
	.idProduct =		cpu_to_le16(FSG_PRODUCT_ID),
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
#define STRING_CONFIGURATION_IDX	2

static char manufacturer[50];

static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer,
	[STRING_PRODUCT_IDX].s = DRIVER_DESC,
	[STRING_CONFIGURATION_IDX].s = "Self Powered",
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



/****************************** Configurations ******************************/

static struct fsg_module_parameters mod_data = {
	.stall = 1
};
FSG_MODULE_PARAMETERS(/* no prefix */, mod_data);

static unsigned long msg_registered = 0;
static void msg_cleanup(void);

static int msg_thread_exits(struct fsg_common *common)
{
	msg_cleanup();
	return 0;
}

static int __init msg_do_config(struct usb_configuration *c)
{
	struct fsg_common *common;
	struct fsg_config config;
	int ret;

	if (gadget_is_otg(c->cdev->gadget)) {
		c->descriptors = otg_desc;
		c->bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	fsg_config_from_params(&config, &mod_data);
	config.thread_exits = msg_thread_exits;
	common = fsg_common_init(0, c->cdev, &config);
	if (IS_ERR(common))
		return PTR_ERR(common);

	ret = fsg_add(c->cdev, c, common);
	fsg_common_put(common);
	return ret;
}

static struct usb_configuration msg_config_driver = {
	.label			= "Linux File-Backed Storage",
	.bind			= msg_do_config,
	.bConfigurationValue	= 1,
	/* .iConfiguration = DYNAMIC */
	.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,
};



/****************************** Gadget Bind ******************************/


static int __init msg_bind(struct usb_composite_dev *cdev)
{
	struct usb_gadget *gadget = cdev->gadget;
	int status;

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */

	/* device descriptor strings: manufacturer, product */
	snprintf(manufacturer, sizeof manufacturer, "%s %s with %s",
	         init_utsname()->sysname, init_utsname()->release,
	         gadget->name);
	status = usb_string_id(cdev);
	if (status < 0)
		return status;
	strings_dev[STRING_MANUFACTURER_IDX].id = status;
	msg_device_desc.iManufacturer = status;

	status = usb_string_id(cdev);
	if (status < 0)
		return status;
	strings_dev[STRING_PRODUCT_IDX].id = status;
	msg_device_desc.iProduct = status;

	status = usb_string_id(cdev);
	if (status < 0)
		return status;
	strings_dev[STRING_CONFIGURATION_IDX].id = status;
	msg_config_driver.iConfiguration = status;

	/* register our second configuration */
	status = usb_add_config(cdev, &msg_config_driver);
	if (status < 0)
		return status;

	dev_info(&gadget->dev, DRIVER_DESC ", version: " DRIVER_VERSION "\n");
	set_bit(0, &msg_registered);
	return 0;
}


/****************************** Some noise ******************************/


static struct usb_composite_driver msg_driver = {
	.name		= "g_mass_storage",
	.dev		= &msg_device_desc,
	.strings	= dev_strings,
	.bind		= msg_bind,
};

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Michal Nazarewicz");
MODULE_LICENSE("GPL");

static int __init msg_init(void)
{
	return usb_composite_register(&msg_driver);
}
module_init(msg_init);

static void msg_cleanup(void)
{
	if (test_and_clear_bit(0, &msg_registered))
		usb_composite_unregister(&msg_driver);
}
module_exit(msg_cleanup);
