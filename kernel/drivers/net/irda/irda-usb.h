

#include <linux/time.h>

#include <net/irda/irda.h>
#include <net/irda/irda_device.h>      /* struct irlap_cb */

#define RX_COPY_THRESHOLD 200
#define IRDA_USB_MAX_MTU 2051
#define IRDA_USB_SPEED_MTU 64		/* Weird, but work like this */

#define IU_MAX_ACTIVE_RX_URBS	1	/* Don't touch !!! */

#define IU_MAX_RX_URBS	(IU_MAX_ACTIVE_RX_URBS + 1)

/* Various ugly stuff to try to workaround generic problems */
/* Send speed command in case of timeout, just for trying to get things sane */
#define IU_BUG_KICK_TIMEOUT
/* Show the USB class descriptor */
#undef IU_DUMP_CLASS_DESC 
#define IU_USB_MIN_RTT		500	/* This should be safe in most cases */

/* Inbound header */
#define MEDIA_BUSY    0x80

#define SPEED_2400     0x01
#define SPEED_9600     0x02
#define SPEED_19200    0x03
#define SPEED_38400    0x04
#define SPEED_57600    0x05
#define SPEED_115200   0x06
#define SPEED_576000   0x07
#define SPEED_1152000  0x08
#define SPEED_4000000  0x09
#define SPEED_16000000 0x0a

/* Basic capabilities */
#define IUC_DEFAULT	0x00	/* Basic device compliant with 1.0 spec */
/* Main bugs */
#define IUC_SPEED_BUG	0x01	/* Device doesn't set speed after the frame */
#define IUC_NO_WINDOW	0x02	/* Device doesn't behave with big Rx window */
#define IUC_NO_TURN	0x04	/* Device doesn't do turnaround by itself */
/* Not currently used */
#define IUC_SIR_ONLY	0x08	/* Device doesn't behave at FIR speeds */
#define IUC_SMALL_PKT	0x10	/* Device doesn't behave with big Rx packets */
#define IUC_MAX_WINDOW	0x20	/* Device underestimate the Rx window */
#define IUC_MAX_XBOFS	0x40	/* Device need more xbofs than advertised */
#define IUC_STIR421X	0x80	/* SigmaTel 4210/4220/4116 VFIR */

/* USB class definitions */
#define USB_IRDA_HEADER            0x01
#define USB_CLASS_IRDA             0x02 /* USB_CLASS_APP_SPEC subclass */
#define USB_DT_IRDA                0x21
#define USB_IRDA_STIR421X_HEADER   0x03
#define IU_SIGMATEL_MAX_RX_URBS    (IU_MAX_ACTIVE_RX_URBS + \
                                    USB_IRDA_STIR421X_HEADER)

struct irda_class_desc {
	__u8  bLength;
	__u8  bDescriptorType;
	__le16 bcdSpecRevision;
	__u8  bmDataSize;
	__u8  bmWindowSize;
	__u8  bmMinTurnaroundTime;
	__le16 wBaudRate;
	__u8  bmAdditionalBOFs;
	__u8  bIrdaRateSniff;
	__u8  bMaxUnicastList;
} __attribute__ ((packed));


#define IU_REQ_GET_CLASS_DESC	0x06
#define STIR421X_MAX_PATCH_DOWNLOAD_SIZE 1023

struct irda_usb_cb {
	struct irda_class_desc *irda_desc;
	struct usb_device *usbdev;	/* init: probe_irda */
	struct usb_interface *usbintf;	/* init: probe_irda */
	int netopen;			/* Device is active for network */
	int present;			/* Device is present on the bus */
	__u32 capability;		/* Capability of the hardware */
	__u8  bulk_in_ep;		/* Rx Endpoint assignments */
	__u8  bulk_out_ep;		/* Tx Endpoint assignments */
	__u16 bulk_out_mtu;		/* Max Tx packet size in bytes */
	__u8  bulk_int_ep;		/* Interrupt Endpoint assignments */

	__u8  max_rx_urb;
	struct urb **rx_urb;	        /* URBs used to receive data frames */
	struct urb *idle_rx_urb;	/* Pointer to idle URB in Rx path */
	struct urb *tx_urb;		/* URB used to send data frames */
	struct urb *speed_urb;		/* URB used to send speed commands */
	
	struct net_device *netdev;	/* Yes! we are some kind of netdev. */
	struct irlap_cb   *irlap;	/* The link layer we are binded to */
	struct qos_info qos;
	char *speed_buff;		/* Buffer for speed changes */
	char *tx_buff;

	struct timeval stamp;
	struct timeval now;

	spinlock_t lock;		/* For serializing Tx operations */

	__u16 xbofs;			/* Current xbofs setting */
	__s16 new_xbofs;		/* xbofs we need to set */
	__u32 speed;			/* Current speed */
	__s32 new_speed;		/* speed we need to set */

	__u8 header_length;             /* USB-IrDA frame header size */
	int needspatch;        		/* device needs firmware patch */

	struct timer_list rx_defer_timer;	/* Wait for Rx error to clear */
};

