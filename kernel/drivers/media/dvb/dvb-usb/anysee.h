

#ifndef _DVB_USB_ANYSEE_H_
#define _DVB_USB_ANYSEE_H_

#define DVB_USB_LOG_PREFIX "anysee"
#include "dvb-usb.h"

#define deb_info(args...) dprintk(dvb_usb_anysee_debug, 0x01, args)
#define deb_xfer(args...) dprintk(dvb_usb_anysee_debug, 0x02, args)
#define deb_rc(args...)   dprintk(dvb_usb_anysee_debug, 0x04, args)
#define deb_reg(args...)  dprintk(dvb_usb_anysee_debug, 0x08, args)
#define deb_i2c(args...)  dprintk(dvb_usb_anysee_debug, 0x10, args)
#define deb_fw(args...)   dprintk(dvb_usb_anysee_debug, 0x20, args)

enum cmd {
	CMD_I2C_READ            = 0x33,
	CMD_I2C_WRITE           = 0x31,
	CMD_REG_READ            = 0xb0,
	CMD_REG_WRITE           = 0xb1,
	CMD_STREAMING_CTRL      = 0x12,
	CMD_LED_AND_IR_CTRL     = 0x16,
	CMD_GET_IR_CODE         = 0x41,
	CMD_GET_HW_INFO         = 0x19,
	CMD_SMARTCARD           = 0x34,
};

struct anysee_state {
	u8 tuner;
	u8 seq;
};

#endif

