


#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>

#include "hid-ids.h"

static void mr_report_fixup(struct hid_device *hdev, __u8 *rdesc,
		unsigned int rsize)
{
	if (rsize >= 30 && rdesc[29] == 0x05 && rdesc[30] == 0x09) {
		dev_info(&hdev->dev, "fixing up button/consumer in HID report "
				"descriptor\n");
		rdesc[30] = 0x0c;
	}
}

#define mr_map_key_clear(c)	hid_map_usage_clear(hi, usage, bit, max, \
					EV_KEY, (c))
static int mr_input_mapping(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{
	if ((usage->hid & HID_USAGE_PAGE) != HID_UP_CONSUMER)
		return 0;

	switch (usage->hid & HID_USAGE) {
	case 0x156: mr_map_key_clear(KEY_WORDPROCESSOR);	break;
	case 0x157: mr_map_key_clear(KEY_SPREADSHEET);		break;
	case 0x158: mr_map_key_clear(KEY_PRESENTATION);		break;
	case 0x15c: mr_map_key_clear(KEY_STOP);			break;
	default:
		return 0;
	}
	return 1;
}

static const struct hid_device_id mr_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_MONTEREY, USB_DEVICE_ID_GENIUS_KB29E) },
	{ }
};
MODULE_DEVICE_TABLE(hid, mr_devices);

static struct hid_driver mr_driver = {
	.name = "monterey",
	.id_table = mr_devices,
	.report_fixup = mr_report_fixup,
	.input_mapping = mr_input_mapping,
};

static int __init mr_init(void)
{
	return hid_register_driver(&mr_driver);
}

static void __exit mr_exit(void)
{
	hid_unregister_driver(&mr_driver);
}

module_init(mr_init);
module_exit(mr_exit);
MODULE_LICENSE("GPL");
