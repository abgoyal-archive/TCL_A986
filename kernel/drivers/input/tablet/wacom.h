

#ifndef WACOM_H
#define WACOM_H
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/init.h>
#include <linux/usb/input.h>
#include <asm/unaligned.h>

#define DRIVER_VERSION "v1.52"
#define DRIVER_AUTHOR "Vojtech Pavlik <vojtech@ucw.cz>"
#define DRIVER_DESC "USB Wacom tablet driver"
#define DRIVER_LICENSE "GPL"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);

#define USB_VENDOR_ID_WACOM	0x056a

struct wacom {
	dma_addr_t data_dma;
	struct usb_device *usbdev;
	struct usb_interface *intf;
	struct urb *irq;
	struct wacom_wac wacom_wac;
	struct mutex lock;
	bool open;
	char phys[32];
};

extern const struct usb_device_id wacom_ids[];

void wacom_wac_irq(struct wacom_wac *wacom_wac, size_t len);
void wacom_setup_input_capabilities(struct input_dev *input_dev,
				    struct wacom_wac *wacom_wac);
#endif
