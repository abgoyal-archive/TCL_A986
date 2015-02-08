
#ifndef __USB_H
#define __USB_H

#include <linux/usb.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/leds.h>
#include <net/cfg80211.h>
#include <net/mac80211.h>
#include <linux/firmware.h>
#include "eeprom.h"
#include "hw.h"
#include "ar9170.h"

#define AR9170_NUM_RX_URBS			16
#define AR9170_NUM_TX_URBS			8

struct firmware;

struct ar9170_usb {
	struct ar9170 common;
	struct usb_device *udev;
	struct usb_interface *intf;

	struct usb_anchor rx_submitted;
	struct usb_anchor tx_pending;
	struct usb_anchor tx_submitted;

	bool req_one_stage_fw;

	spinlock_t tx_urb_lock;
	atomic_t tx_submitted_urbs;
	unsigned int tx_pending_urbs;

	struct completion cmd_wait;
	struct completion firmware_loading_complete;
	int readlen;
	u8 *readbuf;

	const struct firmware *init_values;
	const struct firmware *firmware;
};

#endif /* __USB_H */
