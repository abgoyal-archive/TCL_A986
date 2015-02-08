

#ifndef _DVB_USB_CE6230_H_
#define _DVB_USB_CE6230_H_

#define DVB_USB_LOG_PREFIX "ce6230"
#include "dvb-usb.h"

#define deb_info(args...) dprintk(dvb_usb_ce6230_debug, 0x01, args)
#define deb_rc(args...)   dprintk(dvb_usb_ce6230_debug, 0x02, args)
#define deb_xfer(args...) dprintk(dvb_usb_ce6230_debug, 0x04, args)
#define deb_reg(args...)  dprintk(dvb_usb_ce6230_debug, 0x08, args)
#define deb_i2c(args...)  dprintk(dvb_usb_ce6230_debug, 0x10, args)
#define deb_fw(args...)   dprintk(dvb_usb_ce6230_debug, 0x20, args)

#define ce6230_debug_dump(r, t, v, i, b, l, func) { \
	int loop_; \
	func("%02x %02x %02x %02x %02x %02x %02x %02x", \
		t, r, v & 0xff, v >> 8, i & 0xff, i >> 8, l & 0xff, l >> 8); \
	if (t == (USB_TYPE_VENDOR | USB_DIR_OUT)) \
		func(" >>> "); \
	else \
		func(" <<< "); \
	for (loop_ = 0; loop_ < l; loop_++) \
		func("%02x ", b[loop_]); \
	func("\n");\
}

#define CE6230_USB_TIMEOUT 1000

struct req_t {
	u8  cmd;       /* [1] */
	u16 value;     /* [2|3] */
	u16 index;     /* [4|5] */
	u16 data_len;  /* [6|7] */
	u8  *data;
};

enum ce6230_cmd {
	CONFIG_READ          = 0xd0, /* rd 0 (unclear) */
	UNKNOWN_WRITE        = 0xc7, /* wr 7 (unclear) */
	I2C_READ             = 0xd9, /* rd 9 (unclear) */
	I2C_WRITE            = 0xca, /* wr a */
	DEMOD_READ           = 0xdb, /* rd b */
	DEMOD_WRITE          = 0xcc, /* wr c */
	REG_READ             = 0xde, /* rd e */
	REG_WRITE            = 0xcf, /* wr f */
};

#endif
