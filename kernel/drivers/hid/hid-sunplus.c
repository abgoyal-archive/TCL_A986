


#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>

#include "hid-ids.h"

static void sp_report_fixup(struct hid_device *hdev, __u8 *rdesc,
		unsigned int rsize)
{
	if (rsize >= 107 && rdesc[104] == 0x26 && rdesc[105] == 0x80 &&
			rdesc[106] == 0x03) {
		dev_info(&hdev->dev, "fixing up Sunplus Wireless Desktop "
				"report descriptor\n");
		rdesc[105] = rdesc[110] = 0x03;
		rdesc[106] = rdesc[111] = 0x21;
	}
}

#define sp_map_key_clear(c)	hid_map_usage_clear(hi, usage, bit, max, \
		EV_KEY, (c))
static int sp_input_mapping(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{
	if ((usage->hid & HID_USAGE_PAGE) != HID_UP_CONSUMER)
		return 0;

	switch (usage->hid & HID_USAGE) {
	case 0x2003: sp_map_key_clear(KEY_ZOOMIN);		break;
	case 0x2103: sp_map_key_clear(KEY_ZOOMOUT);	break;
	default:
		return 0;
	}
	return 1;
}

static const struct hid_device_id sp_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_SUNPLUS, USB_DEVICE_ID_SUNPLUS_WDESKTOP) },
	{ }
};
MODULE_DEVICE_TABLE(hid, sp_devices);

static struct hid_driver sp_driver = {
	.name = "sunplus",
	.id_table = sp_devices,
	.report_fixup = sp_report_fixup,
	.input_mapping = sp_input_mapping,
};

static int __init sp_init(void)
{
	return hid_register_driver(&sp_driver);
}

static void __exit sp_exit(void)
{
	hid_unregister_driver(&sp_driver);
}

module_init(sp_init);
module_exit(sp_exit);
MODULE_LICENSE("GPL");
