


#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>

#include "hid-ids.h"

static void ch_report_fixup(struct hid_device *hdev, __u8 *rdesc,
		unsigned int rsize)
{
	if (rsize >= 17 && rdesc[11] == 0x3c && rdesc[12] == 0x02) {
		dev_info(&hdev->dev, "fixing up Cherry Cymotion report "
				"descriptor\n");
		rdesc[11] = rdesc[16] = 0xff;
		rdesc[12] = rdesc[17] = 0x03;
	}
}

#define ch_map_key_clear(c)	hid_map_usage_clear(hi, usage, bit, max, \
					EV_KEY, (c))
static int ch_input_mapping(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{
	if ((usage->hid & HID_USAGE_PAGE) != HID_UP_CONSUMER)
		return 0;

	switch (usage->hid & HID_USAGE) {
	case 0x301: ch_map_key_clear(KEY_PROG1);	break;
	case 0x302: ch_map_key_clear(KEY_PROG2);	break;
	case 0x303: ch_map_key_clear(KEY_PROG3);	break;
	default:
		return 0;
	}

	return 1;
}

static const struct hid_device_id ch_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_CHERRY, USB_DEVICE_ID_CHERRY_CYMOTION) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_CHERRY, USB_DEVICE_ID_CHERRY_CYMOTION_SOLAR) },
	{ }
};
MODULE_DEVICE_TABLE(hid, ch_devices);

static struct hid_driver ch_driver = {
	.name = "cherry",
	.id_table = ch_devices,
	.report_fixup = ch_report_fixup,
	.input_mapping = ch_input_mapping,
};

static int __init ch_init(void)
{
	return hid_register_driver(&ch_driver);
}

static void __exit ch_exit(void)
{
	hid_unregister_driver(&ch_driver);
}

module_init(ch_init);
module_exit(ch_exit);
MODULE_LICENSE("GPL");
