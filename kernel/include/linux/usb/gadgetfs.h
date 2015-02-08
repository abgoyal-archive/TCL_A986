

#ifndef __LINUX_USB_GADGETFS_H
#define __LINUX_USB_GADGETFS_H

#include <linux/types.h>
#include <linux/ioctl.h>

#include <linux/usb/ch9.h>


enum usb_gadgetfs_event_type {
	GADGETFS_NOP = 0,

	GADGETFS_CONNECT,
	GADGETFS_DISCONNECT,
	GADGETFS_SETUP,
	GADGETFS_SUSPEND,
	/* and likely more ! */
};

struct usb_gadgetfs_event {
	union {
		/* NOP, DISCONNECT, SUSPEND: nothing
		 * ... some hardware can't report disconnection
		 */

		/* CONNECT: just the speed */
		enum usb_device_speed	speed;

		/* SETUP: packet; DATA phase i/o precedes next event
		 *(setup.bmRequestType & USB_DIR_IN) flags direction
		 * ... includes SET_CONFIGURATION, SET_INTERFACE
		 */
		struct usb_ctrlrequest	setup;
	} u;
	enum usb_gadgetfs_event_type	type;
};



/* endpoint ioctls */

#define	GADGETFS_FIFO_STATUS	_IO('g', 1)

/* discards any unclaimed data in the fifo. */
#define	GADGETFS_FIFO_FLUSH	_IO('g', 2)

#define	GADGETFS_CLEAR_HALT	_IO('g', 3)

#endif /* __LINUX_USB_GADGETFS_H */
