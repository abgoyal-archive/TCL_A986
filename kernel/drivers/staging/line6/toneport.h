

#ifndef TONEPORT_H
#define TONEPORT_H


#include "driver.h"

#include <linux/usb.h>
#include <sound/core.h>


struct usb_line6_toneport {
	/**
		 Generic Line6 USB data.
	*/
	struct usb_line6 line6;

	/**
		 Serial number of device.
	*/
	int serial_number;

	/**
		 Firmware version (x 100).
	*/
	int firmware_version;
};


extern void toneport_disconnect(struct usb_interface *interface);
extern int toneport_init(struct usb_interface *interface,
			 struct usb_line6_toneport *toneport);


#endif
