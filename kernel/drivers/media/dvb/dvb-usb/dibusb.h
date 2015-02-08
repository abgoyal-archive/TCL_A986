
#ifndef _DVB_USB_DIBUSB_H_
#define _DVB_USB_DIBUSB_H_

#ifndef DVB_USB_LOG_PREFIX
 #define DVB_USB_LOG_PREFIX "dibusb"
#endif
#include "dvb-usb.h"

#include "dib3000.h"
#include "dib3000mc.h"
#include "mt2060.h"



#define DIBUSB_REQ_START_READ			0x00
#define DIBUSB_REQ_START_DEMOD			0x01

#define DIBUSB_REQ_I2C_READ			0x02

#define DIBUSB_REQ_I2C_WRITE			0x03

#define DIBUSB_REQ_POLL_REMOTE       0x04

/* additional status values for Hauppauge Remote Control Protocol */
#define DIBUSB_RC_HAUPPAUGE_KEY_PRESSED	0x01
#define DIBUSB_RC_HAUPPAUGE_KEY_EMPTY	0x03

#define DIBUSB_REQ_SET_STREAMING_MODE	0x05

/* interrupt the internal read loop, when blocking */
#define DIBUSB_REQ_INTR_READ			0x06

#define DIBUSB_REQ_SET_IOCTL			0x07

/* IOCTL commands */

/* change the power mode in firmware */
#define DIBUSB_IOCTL_CMD_POWER_MODE		0x00
#define DIBUSB_IOCTL_POWER_SLEEP			0x00
#define DIBUSB_IOCTL_POWER_WAKEUP			0x01

/* modify streaming of the FX2 */
#define DIBUSB_IOCTL_CMD_ENABLE_STREAM	0x01
#define DIBUSB_IOCTL_CMD_DISABLE_STREAM	0x02

struct dibusb_state {
	struct dib_fe_xfer_ops ops;
	int mt2060_present;
	u8 tuner_addr;
};

struct dibusb_device_state {
	/* for RC5 remote control */
	int old_toggle;
	int last_repeat_count;
};

extern struct i2c_algorithm dibusb_i2c_algo;

extern int dibusb_dib3000mc_frontend_attach(struct dvb_usb_adapter *);
extern int dibusb_dib3000mc_tuner_attach (struct dvb_usb_adapter *);

extern int dibusb_streaming_ctrl(struct dvb_usb_adapter *, int);
extern int dibusb_pid_filter(struct dvb_usb_adapter *, int, u16, int);
extern int dibusb_pid_filter_ctrl(struct dvb_usb_adapter *, int);
extern int dibusb2_0_streaming_ctrl(struct dvb_usb_adapter *, int);

extern int dibusb_power_ctrl(struct dvb_usb_device *, int);
extern int dibusb2_0_power_ctrl(struct dvb_usb_device *, int);

#define DEFAULT_RC_INTERVAL 150
//#define DEFAULT_RC_INTERVAL 100000

extern struct dvb_usb_rc_key ir_codes_dibusb_table[];
extern int dibusb_rc_query(struct dvb_usb_device *, u32 *, int *);
extern int dibusb_read_eeprom_byte(struct dvb_usb_device *, u8, u8 *);

#endif
