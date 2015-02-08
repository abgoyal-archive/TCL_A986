
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/slab.h>

#include <linux/device.h>
#include <linux/errno.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>


#define unk	KEY_UNKNOWN
#define HID_SAY	"HID:"


static unsigned short hid_keycode[256] = {
	  0,  0,  0,  0, 30, 48, 46, 32, 18, 33, 34, 35, 23, 36, 37, 38,
	 50, 49, 24, 25, 16, 19, 31, 20, 22, 47, 17, 45, 21, 44,  2,  3,
	  4,  5,  6,  7,  8,  9, 10, 11, 232, 158, 14, 15, 57, 12, 13, 26,
	 27, 43, 43, 39, 40, 41, 51, 52, 53, 58, 59, 60, 61, 62, 63, 64,
	 65, 66, 67, 68, 87, 88, 99, 70,119,110,102,104,111,107,109,106,
	105,108,103, 69, 98, 55, 74, 78, 96, 79, 80, 81, 75, 76, 77, 71,
	 72, 73, 82, 83, 86,127,116,117,183,184,185,186,187,188,189,190,
	191,192,193,194,134,138,130,132,128,129,131,137,133,135,136,113,
	115,114,unk,unk,unk,121,unk, 89, 93,124, 92, 94, 95,unk,unk,unk,
	122,123, 90, 91, 85,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,
	unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,
	unk,unk,unk,unk,unk,unk,179,180,unk,unk,unk,unk,unk,unk,unk,unk,
	unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,
	unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,unk,
	 29, 42, 56,125, 97, 54,100,126,164,166,165,163,161,115,114,113,
	150,158,159,128,136,177,178,176,142,152,173,140,unk,unk,unk,unk
};

#define HID_KBD_NAME  "hid-keyboard"
#define HID_KEY_PRESS 1
#define HID_KEY_RELEASE 0

static struct input_dev *hid_input_dev;

struct hidkeyboard {
    struct input_dev *input;
    unsigned short keymap[ARRAY_SIZE(hid_keycode)];
};

struct hidkeyboard *hidkbd;


static long hid_kbd_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	unsigned char keycode;
	printk(HID_SAY"hid_kbd_dev_ioctl\n");			
	if (copy_from_user(&keycode, uarg, sizeof(keycode)))
		return -EFAULT;
	switch(cmd)
	{
		case HID_KEY_PRESS:
		{
			printk(HID_SAY"hid keycode  %d press\n",keycode);
			input_report_key(hid_input_dev,hid_keycode[keycode], 1);
			input_sync(hid_input_dev);
			break;
		}
		case HID_KEY_RELEASE:
		{
			printk(HID_SAY"hid keycode %d release\n",keycode);
			input_report_key(hid_input_dev,hid_keycode[keycode], 0);
			input_sync(hid_input_dev);
			break;
		}
		default:
			return -EINVAL;
	}
	return 0;
}

static int hid_kbd_dev_open(struct inode *inode, struct file *file)
{

	int err,i;
	printk(HID_SAY"*** hidkeyboard hid_kbd_dev_open ***\n");

	hidkbd = kzalloc(sizeof(struct hidkeyboard), GFP_KERNEL);
    	hid_input_dev = input_allocate_device();
   	 if (!hidkbd || !hid_input_dev) 
		goto fail;
	
    	memcpy(hidkbd->keymap, hid_keycode,
		sizeof(hid_keycode));
	hidkbd->input = hid_input_dev;

	__set_bit(EV_KEY, hid_input_dev->evbit);

	hid_input_dev->name = HID_KBD_NAME;
	hid_input_dev->keycode = hidkbd->keymap;
	hid_input_dev->keycodesize = sizeof(unsigned short);
	hid_input_dev->keycodemax = ARRAY_SIZE(hid_keycode);
	hid_input_dev->id.bustype = BUS_HOST;

	for (i = 0; i < ARRAY_SIZE(hidkbd->keymap); i++)
		__set_bit(hidkbd->keymap[i], hid_input_dev->keybit);

	input_set_capability(hid_input_dev, EV_MSC, MSC_SCAN);
	err = input_register_device(hid_input_dev);
    	if (err) {
		printk(HID_SAY"register input device failed (%d)\n", err);
		input_free_device(hid_input_dev);
		return err;
	}
	return 0;
fail:
    //platform_set_drvdata(pdev, NULL);
    input_free_device(hid_input_dev);
    kfree(hidkbd);
    
    return -EINVAL;
}

static int hid_kbd_dev_release(struct inode *inode, struct file *file)
{
	printk(HID_SAY"*** hidkeyboard hid_kbd_dev_release ***\n");
	input_unregister_device(hid_input_dev);
	return 0;
}


static struct file_operations hid_kbd_dev_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= hid_kbd_dev_ioctl,
	.open		= hid_kbd_dev_open,
	.release		= hid_kbd_dev_release
};

static struct miscdevice hid_kbd_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= HID_KBD_NAME,
	.fops	= &hid_kbd_dev_fops,
};


static int hid_keyboard_probe(struct platform_device *pdev)
{

    int i,err;
    
    printk(HID_SAY"*** hidkeyboard probe ***\n");

    hidkbd = kzalloc(sizeof(struct hidkeyboard), GFP_KERNEL);
    hid_input_dev = input_allocate_device();
    if (!hidkbd || !hid_input_dev) 
		goto fail;
	
    	memcpy(hidkbd->keymap, hid_keycode,
		sizeof(hid_keycode));
	hidkbd->input = hid_input_dev;
	__set_bit(EV_KEY, hid_input_dev->evbit);
	platform_set_drvdata(pdev, hidkbd);

	hid_input_dev->name = HID_KBD_NAME;
	hid_input_dev->keycode = hidkbd->keymap;
	hid_input_dev->keycodesize = sizeof(unsigned short);
	hid_input_dev->keycodemax = ARRAY_SIZE(hid_keycode);
	hid_input_dev->id.bustype = BUS_HOST;
	hid_input_dev->dev.parent = &pdev->dev;


	for (i = 0; i < ARRAY_SIZE(hidkbd->keymap); i++)
		__set_bit(hidkbd->keymap[i], hid_input_dev->keybit);

	input_set_capability(hid_input_dev, EV_MSC, MSC_SCAN);
	hid_kbd_dev.parent = &pdev->dev;
	err = misc_register(&hid_kbd_dev);
	if (err) {
		printk(HID_SAY"register device failed (%d)\n", err);
		//input_unregister_device(hid_input_dev);
		return err;
	}

    return 0;

fail:
    platform_set_drvdata(pdev, NULL);
    input_free_device(hid_input_dev);
    kfree(hidkbd);
    
    return -EINVAL;
}

static struct platform_driver hid_keyboard_driver = {
    .probe = hid_keyboard_probe,
    .driver = {
        .name = HID_KBD_NAME,
    },
};

static int __devinit hid_keyboard_init(void)
{
	printk(HID_SAY"hid_keyboard_init OK\n");

    return platform_driver_register(&hid_keyboard_driver);
}


static void __exit hid_keyboard_exit(void)
{
}

module_init(hid_keyboard_init);
module_exit(hid_keyboard_exit);

MODULE_DESCRIPTION("hid keyboard Device");
MODULE_LICENSE("GPL");

