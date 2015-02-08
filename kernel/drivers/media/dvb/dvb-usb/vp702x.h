
#ifndef _DVB_USB_VP7021_H_
#define _DVB_USB_VP7021_H_

#define DVB_USB_LOG_PREFIX "vp702x"
#include "dvb-usb.h"

extern int dvb_usb_vp702x_debug;
#define deb_info(args...) dprintk(dvb_usb_vp702x_debug,0x01,args)
#define deb_xfer(args...) dprintk(dvb_usb_vp702x_debug,0x02,args)
#define deb_rc(args...)   dprintk(dvb_usb_vp702x_debug,0x04,args)
#define deb_fe(args...)   dprintk(dvb_usb_vp702x_debug,0x08,args)

/* commands are read and written with USB control messages */

/* consecutive read/write operation */
#define REQUEST_OUT		0xB2
#define REQUEST_IN		0xB3


#define GET_TUNER_STATUS	0x05

#define GET_SYSTEM_STRING	0x06

#define SET_DISEQC_CMD		0x08

#define SET_LNB_POWER		0x09

#define GET_MAC_ADDRESS		0x0A
/* #define GET_MAC_ADDRESS   0x0B */

#define SET_PID_FILTER		0x11


/* one direction requests */
#define READ_REMOTE_REQ		0xB4
/* IN  i: 0; v: 0; b[0] == request, b[1] == key */

#define READ_PID_NUMBER_REQ	0xB5
/* IN  i: 0; v: 0; b[0] == request, b[1] == 0, b[2] = pid number */

#define WRITE_EEPROM_REQ	0xB6
/* OUT i: offset; v: value to write; no extra buffer */

#define READ_EEPROM_REQ		0xB7
/* IN  i: bufferlen; v: offset; buffer with bufferlen bytes */

#define READ_STATUS		0xB8
/* IN  i: 0; v: 0; bufferlen 10 */

#define READ_TUNER_REG_REQ	0xB9
/* IN  i: 0; v: register; b[0] = value */

#define READ_FX2_REG_REQ	0xBA
/* IN  i: offset; v: 0; b[0] = value */

#define WRITE_FX2_REG_REQ	0xBB
/* OUT i: offset; v: value to write; 1 byte extra buffer */

#define SET_TUNER_POWER_REQ	0xBC
/* IN  i: 0 = power off, 1 = power on */

#define WRITE_TUNER_REG_REQ	0xBD
/* IN  i: register, v: value to write, no extra buffer */

#define RESET_TUNER		0xBE
/* IN  i: 0, v: 0, no extra buffer */

extern struct dvb_frontend * vp702x_fe_attach(struct dvb_usb_device *d);

extern int vp702x_usb_inout_op(struct dvb_usb_device *d, u8 *o, int olen, u8 *i, int ilen, int msec);
extern int vp702x_usb_in_op(struct dvb_usb_device *d, u8 req, u16 value, u16 index, u8 *b, int blen);

#endif
