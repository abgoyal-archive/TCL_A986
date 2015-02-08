
#ifndef __WINBOND_WBUSB_S_H
#define __WINBOND_WBUSB_S_H

#include <linux/types.h>

struct wb_usb {
	u32	IsUsb20;
	struct	usb_device *udev;
	u32	DetectCount;
};
#endif
