


#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>

#include "hid-ids.h"

static void ortek_report_fixup(struct hid_device *hdev, __u8 *rdesc,
		unsigned int rsize)
{
	if (rsize >= 56 && rdesc[54] == 0x25 && rdesc[55] == 0x01) {
		dev_info(&hdev->dev, "Fixing up Ortek WKB-2000 "
				"report descriptor.\n");
		rdesc[55] = 0x92;
	}
}

static const struct hid_device_id ortek_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_ORTEK, USB_DEVICE_ID_ORTEK_WKB2000) },
	{ }
};
MODULE_DEVICE_TABLE(hid, ortek_devices);

static struct hid_driver ortek_driver = {
	.name = "ortek",
	.id_table = ortek_devices,
	.report_fixup = ortek_report_fixup
};

static int __init ortek_init(void)
{
	return hid_register_driver(&ortek_driver);
}

static void __exit ortek_exit(void)
{
	hid_unregister_driver(&ortek_driver);
}

module_init(ortek_init);
module_exit(ortek_exit);
MODULE_LICENSE("GPL");
