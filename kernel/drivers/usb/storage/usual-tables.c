

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb_usual.h>


#define UNUSUAL_DEV(id_vendor, id_product, bcdDeviceMin, bcdDeviceMax, \
		    vendorName, productName, useProtocol, useTransport, \
		    initFunction, flags) \
{ USB_DEVICE_VER(id_vendor, id_product, bcdDeviceMin, bcdDeviceMax), \
  .driver_info = (flags)|(USB_US_TYPE_STOR<<24) }

#define COMPLIANT_DEV(id_vendor, id_product, bcdDeviceMin, bcdDeviceMax, \
		    vendorName, productName, useProtocol, useTransport, \
		    initFunction, flags) \
{ USB_DEVICE_VER(id_vendor, id_product, bcdDeviceMin, bcdDeviceMax), \
  .driver_info = (flags) }

#define USUAL_DEV(useProto, useTrans, useType) \
{ USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, useProto, useTrans), \
  .driver_info = ((useType)<<24) }

struct usb_device_id usb_storage_usb_ids[] = {
#	include "unusual_devs.h"
	{ }		/* Terminating entry */
};
EXPORT_SYMBOL_GPL(usb_storage_usb_ids);

MODULE_DEVICE_TABLE(usb, usb_storage_usb_ids);

#undef UNUSUAL_DEV
#undef COMPLIANT_DEV
#undef USUAL_DEV


struct ignore_entry {
	u16	vid, pid, bcdmin, bcdmax;
};

#define UNUSUAL_DEV(id_vendor, id_product, bcdDeviceMin, bcdDeviceMax, \
		    vendorName, productName, useProtocol, useTransport, \
		    initFunction, flags) \
{					\
	.vid	= id_vendor,		\
	.pid 	= id_product,		\
	.bcdmin	= bcdDeviceMin,		\
	.bcdmax = bcdDeviceMax,		\
}

static struct ignore_entry ignore_ids[] = {
#	include "unusual_alauda.h"
#	include "unusual_cypress.h"
#	include "unusual_datafab.h"
#	include "unusual_freecom.h"
#	include "unusual_isd200.h"
#	include "unusual_jumpshot.h"
#	include "unusual_karma.h"
#	include "unusual_onetouch.h"
#	include "unusual_sddr09.h"
#	include "unusual_sddr55.h"
#	include "unusual_usbat.h"
	{ }		/* Terminating entry */
};

#undef UNUSUAL_DEV


/* Return an error if a device is in the ignore_ids list */
int usb_usual_ignore_device(struct usb_interface *intf)
{
	struct usb_device *udev;
	unsigned vid, pid, bcd;
	struct ignore_entry *p;

	udev = interface_to_usbdev(intf);
	vid = le16_to_cpu(udev->descriptor.idVendor);
	pid = le16_to_cpu(udev->descriptor.idProduct);
	bcd = le16_to_cpu(udev->descriptor.bcdDevice);

	for (p = ignore_ids; p->vid; ++p) {
		if (p->vid == vid && p->pid == pid &&
				p->bcdmin <= bcd && p->bcdmax >= bcd)
			return -ENXIO;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(usb_usual_ignore_device);
