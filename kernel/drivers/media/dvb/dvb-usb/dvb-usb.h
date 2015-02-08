
#ifndef __DVB_USB_H__
#define __DVB_USB_H__

#include <linux/input.h>
#include <linux/usb.h>
#include <linux/firmware.h>
#include <linux/mutex.h>

#include "dvb_frontend.h"
#include "dvb_demux.h"
#include "dvb_net.h"
#include "dmxdev.h"

#include "dvb-pll.h"

#include "dvb-usb-ids.h"

/* debug */
#ifdef CONFIG_DVB_USB_DEBUG
#define dprintk(var,level,args...) \
	    do { if ((var & level)) { printk(args); } } while (0)

#define debug_dump(b,l,func) {\
	int loop_; \
	for (loop_ = 0; loop_ < l; loop_++) func("%02x ", b[loop_]); \
	func("\n");\
}
#define DVB_USB_DEBUG_STATUS
#else
#define dprintk(args...)
#define debug_dump(b,l,func)

#define DVB_USB_DEBUG_STATUS " (debugging is not enabled)"

#endif

/* generic log methods - taken from usb.h */
#ifndef DVB_USB_LOG_PREFIX
 #define DVB_USB_LOG_PREFIX "dvb-usb (please define a log prefix)"
#endif

#undef err
#define err(format, arg...)  printk(KERN_ERR     DVB_USB_LOG_PREFIX ": " format "\n" , ## arg)
#undef info
#define info(format, arg...) printk(KERN_INFO    DVB_USB_LOG_PREFIX ": " format "\n" , ## arg)
#undef warn
#define warn(format, arg...) printk(KERN_WARNING DVB_USB_LOG_PREFIX ": " format "\n" , ## arg)

struct dvb_usb_device_description {
	const char *name;

#define DVB_USB_ID_MAX_NUM 15
	struct usb_device_id *cold_ids[DVB_USB_ID_MAX_NUM];
	struct usb_device_id *warm_ids[DVB_USB_ID_MAX_NUM];
};

struct dvb_usb_rc_key {
	u16 scan;
	u32 event;
};

static inline u8 rc5_custom(struct dvb_usb_rc_key *key)
{
	return (key->scan >> 8) & 0xff;
}

static inline u8 rc5_data(struct dvb_usb_rc_key *key)
{
	return key->scan & 0xff;
}

static inline u8 rc5_scan(struct dvb_usb_rc_key *key)
{
	return key->scan & 0xffff;
}

struct dvb_usb_device;
struct dvb_usb_adapter;
struct usb_data_stream;

struct usb_data_stream_properties {
#define USB_BULK  1
#define USB_ISOC  2
	int type;
	int count;
	int endpoint;

	union {
		struct {
			int buffersize; /* per URB */
		} bulk;
		struct {
			int framesperurb;
			int framesize;
			int interval;
		} isoc;
	} u;
};

struct dvb_usb_adapter_properties {
#define DVB_USB_ADAP_HAS_PID_FILTER               0x01
#define DVB_USB_ADAP_PID_FILTER_CAN_BE_TURNED_OFF 0x02
#define DVB_USB_ADAP_NEED_PID_FILTERING           0x04
#define DVB_USB_ADAP_RECEIVES_204_BYTE_TS         0x08
	int caps;
	int pid_filter_count;

	int (*streaming_ctrl)  (struct dvb_usb_adapter *, int);
	int (*pid_filter_ctrl) (struct dvb_usb_adapter *, int);
	int (*pid_filter)      (struct dvb_usb_adapter *, int, u16, int);

	int (*frontend_attach) (struct dvb_usb_adapter *);
	int (*tuner_attach)    (struct dvb_usb_adapter *);

	struct usb_data_stream_properties stream;

	int size_of_priv;

	int (*fe_ioctl_override) (struct dvb_frontend *,
				  unsigned int, void *, unsigned int);
};

#define MAX_NO_OF_ADAPTER_PER_DEVICE 2
struct dvb_usb_device_properties {

#define DVB_USB_IS_AN_I2C_ADAPTER            0x01
	int caps;

#define DEVICE_SPECIFIC 0
#define CYPRESS_AN2135  1
#define CYPRESS_AN2235  2
#define CYPRESS_FX2     3
	int        usb_ctrl;
	int        (*download_firmware) (struct usb_device *, const struct firmware *);
	const char *firmware;
	int        no_reconnect;

	int size_of_priv;

	int num_adapters;
	struct dvb_usb_adapter_properties adapter[MAX_NO_OF_ADAPTER_PER_DEVICE];

	int (*power_ctrl)       (struct dvb_usb_device *, int);
	int (*read_mac_address) (struct dvb_usb_device *, u8 []);
	int (*identify_state)   (struct usb_device *, struct dvb_usb_device_properties *,
			struct dvb_usb_device_description **, int *);

/* remote control properties */
#define REMOTE_NO_KEY_PRESSED      0x00
#define REMOTE_KEY_PRESSED         0x01
#define REMOTE_KEY_REPEAT          0x02
	struct dvb_usb_rc_key  *rc_key_map;
	int rc_key_map_size;
	int (*rc_query) (struct dvb_usb_device *, u32 *, int *);
	int rc_interval;

	struct i2c_algorithm *i2c_algo;

	int generic_bulk_ctrl_endpoint;
	int generic_bulk_ctrl_endpoint_response;

	int num_device_descs;
	struct dvb_usb_device_description devices[12];
};

#define MAX_NO_URBS_FOR_DATA_STREAM 10
struct usb_data_stream {
	struct usb_device                 *udev;
	struct usb_data_stream_properties  props;

#define USB_STATE_INIT    0x00
#define USB_STATE_URB_BUF 0x01
	int state;

	void (*complete) (struct usb_data_stream *, u8 *, size_t);

	struct urb    *urb_list[MAX_NO_URBS_FOR_DATA_STREAM];
	int            buf_num;
	unsigned long  buf_size;
	u8            *buf_list[MAX_NO_URBS_FOR_DATA_STREAM];
	dma_addr_t     dma_addr[MAX_NO_URBS_FOR_DATA_STREAM];

	int urbs_initialized;
	int urbs_submitted;

	void *user_priv;
};

struct dvb_usb_adapter {
	struct dvb_usb_device *dev;
	struct dvb_usb_adapter_properties props;

#define DVB_USB_ADAP_STATE_INIT 0x000
#define DVB_USB_ADAP_STATE_DVB  0x001
	int state;

	u8  id;

	int feedcount;
	int pid_filtering;

	/* dvb */
	struct dvb_adapter   dvb_adap;
	struct dmxdev        dmxdev;
	struct dvb_demux     demux;
	struct dvb_net       dvb_net;
	struct dvb_frontend *fe;
	int                  max_feed_count;

	int (*fe_init)  (struct dvb_frontend *);
	int (*fe_sleep) (struct dvb_frontend *);

	struct usb_data_stream stream;

	void *priv;
};

struct dvb_usb_device {
	struct dvb_usb_device_properties props;
	struct dvb_usb_device_description *desc;

	struct usb_device *udev;

#define DVB_USB_STATE_INIT        0x000
#define DVB_USB_STATE_I2C         0x001
#define DVB_USB_STATE_DVB         0x002
#define DVB_USB_STATE_REMOTE      0x004
	int state;

	int powered;

	/* locking */
	struct mutex usb_mutex;

	/* i2c */
	struct mutex i2c_mutex;
	struct i2c_adapter i2c_adap;

	int                    num_adapters_initialized;
	struct dvb_usb_adapter adapter[MAX_NO_OF_ADAPTER_PER_DEVICE];

	/* remote control */
	struct input_dev *rc_input_dev;
	char rc_phys[64];
	struct delayed_work rc_query_work;
	u32 last_event;
	int last_state;

	struct module *owner;

	void *priv;
};

extern int dvb_usb_device_init(struct usb_interface *,
			       struct dvb_usb_device_properties *,
			       struct module *, struct dvb_usb_device **,
			       short *adapter_nums);
extern void dvb_usb_device_exit(struct usb_interface *);

/* the generic read/write method for device control */
extern int dvb_usb_generic_rw(struct dvb_usb_device *, u8 *, u16, u8 *, u16,int);
extern int dvb_usb_generic_write(struct dvb_usb_device *, u8 *, u16);

/* commonly used remote control parsing */
extern int dvb_usb_nec_rc_key_to_event(struct dvb_usb_device *, u8[], u32 *, int *);

/* commonly used firmware download types and function */
struct hexline {
	u8 len;
	u32 addr;
	u8 type;
	u8 data[255];
	u8 chk;
};
extern int usb_cypress_load_firmware(struct usb_device *udev, const struct firmware *fw, int type);
extern int dvb_usb_get_hexline(const struct firmware *fw, struct hexline *hx, int *pos);


#endif
