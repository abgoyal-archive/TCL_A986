

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>

#include <linux/usb/android_composite.h>
#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/proc_fs.h>
#include <mtk_usb_custom.h>
#include "gadget_chips.h"

#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

extern void usb_check_connect(void);
static const char longname[] = "Gadget Android";

struct android_dev {
	struct usb_composite_dev *cdev;
	struct usb_configuration *config;
	int num_products;
	struct android_usb_product *products;
	int num_functions;
	char **functions;

	int product_id;
	int version;
};

static struct android_dev *_android_dev;

/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

/* String Table */
static struct usb_string strings_dev[] = {
	/* These dummy values should be overridden by platform data */
	[STRING_MANUFACTURER_IDX].s = MANUFACTURER_STRING,
	[STRING_PRODUCT_IDX].s = PRODUCT_STRING,
	[STRING_SERIAL_IDX].s = "0123456789ABCDEF",
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.iManufacturer        = STRING_MANUFACTURER_IDX,
	.iProduct             = STRING_PRODUCT_IDX,
	.bNumConfigurations   = 1,
};

static struct list_head _functions = LIST_HEAD_INIT(_functions);
static int _registered_function_count = 0;

static struct android_usb_function *get_function(const char *name)
{
	struct android_usb_function	*f;
	list_for_each_entry(f, &_functions, list) {
		if (!strcmp(name, f->name))
			return f;
	}
	return 0;
}

static void bind_functions(struct android_dev *dev)
{
	struct android_usb_function	*f;
	char **functions = dev->functions;
	int i;

	for (i = 0; i < dev->num_functions; i++) {
		char *name = *functions++;
		f = get_function(name);
		if (f)
			f->bind_config(dev->config);
		else
			printk(KERN_ERR "function %s not found in bind_functions\n", name);
	}
	printk("USB check whether connect\n");
	usb_check_connect();
}

static int android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	printk(KERN_DEBUG "android_bind_config\n");
	dev->config = c;

	/* bind our functions if they have all registered */
	if (_registered_function_count == dev->num_functions)
		bind_functions(dev);

	return 0;
}

static int android_setup_config(struct usb_configuration *c,
		const struct usb_ctrlrequest *ctrl);

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.bind		= android_bind_config,
	.setup		= android_setup_config,
	.bConfigurationValue = 1,
	.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower	= 0xFA, /* 500ma */
};

static int android_setup_config(struct usb_configuration *c,
		const struct usb_ctrlrequest *ctrl)
{
	int i;
	int ret = -EOPNOTSUPP;

	for (i = 0; i < android_config_driver.next_interface_id; i++) {
		if (android_config_driver.interface[i]->setup) {
			ret = android_config_driver.interface[i]->setup(
				android_config_driver.interface[i], ctrl);
			if (ret >= 0)
				return ret;
		}
	}
	return ret;
}

static int product_has_function(struct android_usb_product *p,
		struct usb_function *f)
{
	char **functions = p->functions;
	int count = p->num_functions;
	const char *name = f->name;
	int i;

	for (i = 0; i < count; i++) {
		if (!strcmp(name, *functions++))
			return 1;
	}
	return 0;
}

static int product_matches_functions(struct android_usb_product *p)
{
	struct usb_function		*f;
	list_for_each_entry(f, &android_config_driver.functions, list) {
		if (product_has_function(p, f) == !!f->disabled)
			return 0;
	}
	return 1;
}

static int get_product_id(struct android_dev *dev)
{
	struct android_usb_product *p = dev->products;
	int count = dev->num_products;
	int i;

	if (p) {
		for (i = 0; i < count; i++, p++) {
			if (product_matches_functions(p))
				return p->product_id;
		}
	}
	/* use default product ID */
	return dev->product_id;
}

static int android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum, id, product_id, ret;

	printk(KERN_INFO "android_bind\n");

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

	/* register our configuration */
	ret = usb_add_config(cdev, &android_config_driver);
	if (ret) {
		printk(KERN_ERR "usb_add_config failed\n");
		return ret;
	}

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	usb_gadget_set_selfpowered(gadget);
	dev->cdev = cdev;
	product_id = get_product_id(dev);
	device_desc.idProduct = __constant_cpu_to_le16(product_id);
	cdev->desc.idProduct = device_desc.idProduct;

	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= android_bind,
	.enable_function = android_enable_function,
};

void android_register_function(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;

	printk(KERN_INFO "android_register_function %s\n", f->name);
	list_add_tail(&f->list, &_functions);
	_registered_function_count++;

	/* bind our functions if they have all registered
	 * and the main driver has bound.
	 */
	if (dev && dev->config && _registered_function_count == dev->num_functions)
		bind_functions(dev);
}

int com_enable = 0;
module_param(com_enable,int,0664);
//extern bool is_meta_mode(void); 
//bool is_advanced_meta_mode(void);
extern bool com_is_enable(void);
void android_enable_function(struct usb_function *f, int enable)
{
    /* The extra check is only enabled when usb meta mode is required.
     * And, the ACM function is only required when usb meta mode is required.
     * Hence, the extra check is enabled only when ACM function is enabled.
     */
   
	struct android_dev *dev = _android_dev;
	int disable = !enable;
	int product_id;
    struct usb_function		*func;
#ifdef CONFIG_USB_ANDROID_ACM
    int extra = (com_is_enable()) && (!strcmp(f->name, "adb"));
#else
    int extra = 0;
#endif

	if (extra || (!!f->disabled != disable)) {
		usb_function_set_enabled(f, !disable);

#ifdef CONFIG_USB_ANDROID_RNDIS
		if (!strcmp(f->name, "rndis")) {
			//struct usb_function		*func;

			/* We need to specify the COMM class in the device descriptor
			 * if we are using RNDIS.
			 */
			if (enable)
#ifdef CONFIG_USB_ANDROID_RNDIS_WCEIS
				dev->cdev->desc.bDeviceClass = USB_CLASS_WIRELESS_CONTROLLER;
#else
				dev->cdev->desc.bDeviceClass = USB_CLASS_COMM;
#endif
			else
				dev->cdev->desc.bDeviceClass = USB_CLASS_PER_INTERFACE;
            dev->cdev->desc.bDeviceSubClass =   0;
            dev->cdev->desc.bDeviceProtocol =   0;
			/* Windows does not support other interfaces when RNDIS is enabled,
			 * so we disable UMS and MTP when RNDIS is on.
			 */
			list_for_each_entry(func, &android_config_driver.functions, list) {
				if (!strcmp(func->name, "usb_mass_storage")
					|| !strcmp(func->name, "mtp")) {
					usb_function_set_enabled(func, !enable);
				}
			}
		}
#endif
#ifdef CONFIG_USB_ANDROID_ACM
		if (!strcmp(f->name, "acm")) {
            if (enable) {
    			dev->cdev->desc.bDeviceClass = USB_CLASS_MISC;
                dev->cdev->desc.bDeviceSubClass =	2;
                dev->cdev->desc.bDeviceProtocol =	1;                
            } else {
                dev->cdev->desc.bDeviceClass = USB_CLASS_PER_INTERFACE;
                dev->cdev->desc.bDeviceSubClass =   0;
                dev->cdev->desc.bDeviceProtocol =   0;
            }
        }
        { /*post processing for avoid some combination*/
            struct usb_function		*acm = NULL, *adb = NULL, *ums = NULL, *rndis = NULL;
            list_for_each_entry(func, &android_config_driver.functions, list) {
				if (!strcmp(func->name, "usb_mass_storage")) 
                    ums = func;
                else if (!strcmp(func->name, "acm"))
                    acm = func;
                else if (!strcmp(func->name, "adb"))
                    adb = func;
                else if (!strcmp(func->name, "rndis"))
                    rndis = func;
            }
		    if (!strcmp(f->name, "acm")) {  
                if (enable) {                       /*enable adb while enabling acm*/
                    adb->disabled = 0;
                    if (!rndis->disabled) {           /*don't enable acm if rndis is enable*/
                        acm->disabled = 1;
                        return;
                    }
                }
		    } else if (!strcmp(f->name, "adb")) {   
		        if (com_is_enable()) {                /*always enable adb in meta mode*/
                    adb->disabled = 0;
                    acm->disabled = 0;
                }
		        else if (!enable)                   /*disable acm while disabling adb*/
                    acm->disabled = 1;
		    } else if (!strcmp(f->name, "rndis")) { /*disable acm while enabling rndis*/
		        if (enable)
		        	{
		        		acm->disabled = 1;
		        		if(!adb->disabled&&(com_enable == 1))
								{
					    			dev->cdev->desc.bDeviceClass = USB_CLASS_MISC;
		                dev->cdev->desc.bDeviceSubClass =	2;
		                dev->cdev->desc.bDeviceProtocol =	1;   
		                printk("rnids and adb both enable\n");
								}
		        	}
                    
		    }
        }
#endif
#ifdef CONFIG_USB_DYNAMIC_INTERFACE
        {   /*update interface count*/
            int intf_count = 0;
			list_for_each_entry(func, &android_config_driver.functions, list) {
		        if (func->disabled)
                    continue;
                else if (!func->update_intf_num)
                    intf_count++;
                else
                    intf_count = func->update_intf_num(func, intf_count);
			}
        }
#endif

		product_id = get_product_id(dev);
		device_desc.idProduct = __constant_cpu_to_le16(product_id);
		if (dev->cdev)
			dev->cdev->desc.idProduct = device_desc.idProduct;
        printk("[USB] ID = %4x, [%02x %02x %02X]\n", device_desc.idProduct, 
               dev->cdev->desc.bDeviceClass, dev->cdev->desc.bDeviceSubClass,dev->cdev->desc.bDeviceProtocol);
		usb_composite_force_reset(dev->cdev);
	}
}
/*----------------------------------------------------------------------------*/
/* procfs: usb serial number                                                  */
/*----------------------------------------------------------------------------*/
static char usb_sn[64] = {0};
static int usbsn_read(char *page, char **start, off_t off,
			          int count, int *eof, void *data)
{
	return scnprintf(page, PAGE_SIZE, "%s\n", strings_dev[STRING_SERIAL_IDX].s);	       
}
static int usbsn_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    char buf[80]; 
    int maxlen;

    maxlen = (count < (sizeof(buf) - 1)) ? count : (sizeof(buf) - 1);
    if(copy_from_user(buf, buffer, maxlen))
		return 0;
    buf[maxlen] = '\0';

    if (1 != sscanf(buf, "%s", usb_sn)) 
        printk(KERN_ERR "write sn fail\n");
    else 
        strings_dev[STRING_SERIAL_IDX].s = usb_sn;
    if (strncmp(usb_sn, "MT-", 3)) /*can't find correct prefix, restore*/
        strings_dev[STRING_SERIAL_IDX].s = "0123456789ABCDEF";
    return count;
}
/*----------------------------------------------------------------------------*/
/* procfs: vendor id                                                          */
/*----------------------------------------------------------------------------*/
static int vendid_read(char *page, char **start, off_t off,
			          int count, int *eof, void *data)
{
	return scnprintf(page, PAGE_SIZE, "%04x\n", device_desc.idVendor);	       
}
static int vendid_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    char buf[80]; 
    int maxlen, id;
    struct android_dev *dev = _android_dev;

    maxlen = (count < (sizeof(buf) - 1)) ? count : (sizeof(buf) - 1);
    if(copy_from_user(buf, buffer, maxlen))
		return 0;
    buf[maxlen] = '\0';

    if (1 != sscanf(buf, "%04x", &id)) 
        printk(KERN_ERR "write vendor id fail\n");
    else {
        device_desc.idVendor = __constant_cpu_to_le16(id);
        dev->cdev->desc.idVendor = device_desc.idVendor;
    }
    return count;
}


static int android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;

	printk(KERN_INFO "android_probe pdata: %p\n", pdata);

	if (pdata) {
		dev->products = pdata->products;
		dev->num_products = pdata->num_products;
		dev->functions = pdata->functions;
		dev->num_functions = pdata->num_functions;
		if (pdata->vendor_id)
			device_desc.idVendor =
				__constant_cpu_to_le16(pdata->vendor_id);
		if (pdata->product_id) {
			dev->product_id = pdata->product_id;
			device_desc.idProduct =
				__constant_cpu_to_le16(pdata->product_id);
		}
		if (pdata->version)
			dev->version = pdata->version;

		if (pdata->product_name)
			strings_dev[STRING_PRODUCT_IDX].s = pdata->product_name;
		if (pdata->manufacturer_name)
			strings_dev[STRING_MANUFACTURER_IDX].s =
					pdata->manufacturer_name;
		if (pdata->serial_number)
			strings_dev[STRING_SERIAL_IDX].s = pdata->serial_number;
		else {
			struct proc_dir_entry *entry = create_proc_entry("usbsn", 0, NULL);
			if (entry) {
				entry->read_proc = usbsn_read;
				entry->write_proc = usbsn_write;
			}
		}
	    {
            struct proc_dir_entry *entry = create_proc_entry("vendid", 0, NULL);
            if (entry) {
                entry->read_proc = vendid_read;
                entry->write_proc = vendid_write;
            }
        }
	}

	return usb_composite_register(&android_usb_driver);
}

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb", },
	.probe = android_probe,
};

static int __init init(void)
{
	struct android_dev *dev;

	printk(KERN_INFO "android init\n");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* set default values, which should be overridden by platform data */
	dev->product_id = PRODUCT_ID;
	_android_dev = dev;

	return platform_driver_register(&android_platform_driver);
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&android_usb_driver);
	platform_driver_unregister(&android_platform_driver);
	kfree(_android_dev);
	_android_dev = NULL;
}
module_exit(cleanup);
