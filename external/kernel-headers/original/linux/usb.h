#ifndef __LINUX_USB_H
#define __LINUX_USB_H

#include <linux/mod_devicetable.h>
#include <linux/usb_ch9.h>

#define USB_MAJOR			180
#define USB_DEVICE_MAJOR		189


#ifdef __KERNEL__

#include <linux/errno.h>        /* for -ENODEV */
#include <linux/delay.h>	/* for mdelay() */
#include <linux/interrupt.h>	/* for in_interrupt() */
#include <linux/list.h>		/* for struct list_head */
#include <linux/kref.h>		/* for struct kref */
#include <linux/device.h>	/* for struct device */
#include <linux/fs.h>		/* for struct file_operations */
#include <linux/completion.h>	/* for struct completion */
#include <linux/sched.h>	/* for current && schedule_timeout */

struct usb_device;
struct usb_driver;

/*-------------------------------------------------------------------------*/


struct ep_device;

struct usb_host_endpoint {
	struct usb_endpoint_descriptor	desc;
	struct list_head		urb_list;
	void				*hcpriv;
	struct ep_device 		*ep_dev;	/* For sysfs info */

	unsigned char *extra;   /* Extra descriptors */
	int extralen;
};

/* host-side wrapper for one interface setting's parsed descriptors */
struct usb_host_interface {
	struct usb_interface_descriptor	desc;

	/* array of desc.bNumEndpoint endpoints associated with this
	 * interface setting.  these will be in no particular order.
	 */
	struct usb_host_endpoint *endpoint;

	char *string;		/* iInterface string, if present */
	unsigned char *extra;   /* Extra descriptors */
	int extralen;
};

enum usb_interface_condition {
	USB_INTERFACE_UNBOUND = 0,
	USB_INTERFACE_BINDING,
	USB_INTERFACE_BOUND,
	USB_INTERFACE_UNBINDING,
};

struct usb_interface {
	/* array of alternate settings for this interface,
	 * stored in no particular order */
	struct usb_host_interface *altsetting;

	struct usb_host_interface *cur_altsetting;	/* the currently
					 * active alternate setting */
	unsigned num_altsetting;	/* number of alternate settings */

	int minor;			/* minor number this interface is
					 * bound to */
	enum usb_interface_condition condition;		/* state of binding */
	struct device dev;		/* interface specific device info */
	struct class_device *class_dev;
};
#define	to_usb_interface(d) container_of(d, struct usb_interface, dev)
#define	interface_to_usbdev(intf) \
	container_of(intf->dev.parent, struct usb_device, dev)

static inline void *usb_get_intfdata (struct usb_interface *intf)
{
	return dev_get_drvdata (&intf->dev);
}

static inline void usb_set_intfdata (struct usb_interface *intf, void *data)
{
	dev_set_drvdata(&intf->dev, data);
}

struct usb_interface *usb_get_intf(struct usb_interface *intf);
void usb_put_intf(struct usb_interface *intf);

/* this maximum is arbitrary */
#define USB_MAXINTERFACES	32

struct usb_interface_cache {
	unsigned num_altsetting;	/* number of alternate settings */
	struct kref ref;		/* reference counter */

	/* variable-length array of alternate settings for this interface,
	 * stored in no particular order */
	struct usb_host_interface altsetting[0];
};
#define	ref_to_usb_interface_cache(r) \
		container_of(r, struct usb_interface_cache, ref)
#define	altsetting_to_usb_interface_cache(a) \
		container_of(a, struct usb_interface_cache, altsetting[0])

struct usb_host_config {
	struct usb_config_descriptor	desc;

	char *string;		/* iConfiguration string, if present */
	/* the interfaces associated with this configuration,
	 * stored in no particular order */
	struct usb_interface *interface[USB_MAXINTERFACES];

	/* Interface information available even when this is not the
	 * active configuration */
	struct usb_interface_cache *intf_cache[USB_MAXINTERFACES];

	unsigned char *extra;   /* Extra descriptors */
	int extralen;
};

int __usb_get_extra_descriptor(char *buffer, unsigned size,
	unsigned char type, void **ptr);
#define usb_get_extra_descriptor(ifpoint,type,ptr)\
	__usb_get_extra_descriptor((ifpoint)->extra,(ifpoint)->extralen,\
		type,(void**)ptr)

/* ----------------------------------------------------------------------- */

struct usb_operations;

/* USB device number allocation bitmap */
struct usb_devmap {
	unsigned long devicemap[128 / (8*sizeof(unsigned long))];
};

struct usb_bus {
	struct device *controller;	/* host/master side hardware */
	int busnum;			/* Bus number (in order of reg) */
	char *bus_name;			/* stable id (PCI slot_name etc) */
	u8 otg_port;			/* 0, or number of OTG/HNP port */
	unsigned is_b_host:1;		/* true during some HNP roleswitches */
	unsigned b_hnp_enable:1;	/* OTG: did A-Host enable HNP? */

	int devnum_next;		/* Next open device number in
					 * round-robin allocation */

	struct usb_devmap devmap;	/* device address allocation map */
	struct usb_operations *op;	/* Operations (specific to the HC) */
	struct usb_device *root_hub;	/* Root hub */
	struct list_head bus_list;	/* list of busses */
	void *hcpriv;                   /* Host Controller private data */

	int bandwidth_allocated;	/* on this bus: how much of the time
					 * reserved for periodic (intr/iso)
					 * requests is used, on average?
					 * Units: microseconds/frame.
					 * Limits: Full/low speed reserve 90%,
					 * while high speed reserves 80%.
					 */
	int bandwidth_int_reqs;		/* number of Interrupt requests */
	int bandwidth_isoc_reqs;	/* number of Isoc. requests */

	struct dentry *usbfs_dentry;	/* usbfs dentry entry for the bus */

	struct class_device *class_dev;	/* class device for this bus */
	struct kref kref;		/* reference counting for this bus */
	void (*release)(struct usb_bus *bus);

#if defined(CONFIG_USB_MON)
	struct mon_bus *mon_bus;	/* non-null when associated */
	int monitored;			/* non-zero when monitored */
#endif
};

/* ----------------------------------------------------------------------- */

#define USB_MAXCHILDREN		(16)

struct usb_tt;

struct usb_device {
	int		devnum;		/* Address on USB bus */
	char		devpath [16];	/* Use in messages: /port/port/... */
	enum usb_device_state	state;	/* configured, not attached, etc */
	enum usb_device_speed	speed;	/* high/full/low (or error) */

	struct usb_tt	*tt; 		/* low/full speed dev, highspeed hub */
	int		ttport;		/* device port on that tt hub */

	unsigned int toggle[2];		/* one bit for each endpoint
					 * ([0] = IN, [1] = OUT) */

	struct usb_device *parent;	/* our hub, unless we're the root */
	struct usb_bus *bus;		/* Bus we're part of */
	struct usb_host_endpoint ep0;

	struct device dev;		/* Generic device interface */

	struct usb_device_descriptor descriptor;/* Descriptor */
	struct usb_host_config *config;	/* All of the configs */

	struct usb_host_config *actconfig;/* the active configuration */
	struct usb_host_endpoint *ep_in[16];
	struct usb_host_endpoint *ep_out[16];

	char **rawdescriptors;		/* Raw descriptors for each config */

	unsigned short bus_mA;		/* Current available from the bus */
	u8 portnum;			/* Parent port number (origin 1) */

	int have_langid;		/* whether string_langid is valid */
	int string_langid;		/* language ID for strings */

	/* static strings from the device */
	char *product;			/* iProduct string, if present */
	char *manufacturer;		/* iManufacturer string, if present */
	char *serial;			/* iSerialNumber string, if present */

	struct list_head filelist;
	struct class_device *class_dev;
	struct dentry *usbfs_dentry;	/* usbfs dentry entry for the device */

	/*
	 * Child devices - these can be either new devices
	 * (if this is a hub device), or different instances
	 * of this same device.
	 *
	 * Each instance needs its own set of data structures.
	 */

	int maxchild;			/* Number of ports if hub */
	struct usb_device *children[USB_MAXCHILDREN];
};
#define	to_usb_device(d) container_of(d, struct usb_device, dev)

extern struct usb_device *usb_get_dev(struct usb_device *dev);
extern void usb_put_dev(struct usb_device *dev);

/* USB device locking */
#define usb_lock_device(udev)		down(&(udev)->dev.sem)
#define usb_unlock_device(udev)		up(&(udev)->dev.sem)
#define usb_trylock_device(udev)	down_trylock(&(udev)->dev.sem)
extern int usb_lock_device_for_reset(struct usb_device *udev,
		struct usb_interface *iface);

/* USB port reset for device reinitialization */
extern int usb_reset_device(struct usb_device *dev);
extern int usb_reset_composite_device(struct usb_device *dev,
		struct usb_interface *iface);

extern struct usb_device *usb_find_device(u16 vendor_id, u16 product_id);

/*-------------------------------------------------------------------------*/

/* for drivers using iso endpoints */
extern int usb_get_current_frame_number (struct usb_device *usb_dev);

/* used these for multi-interface device registration */
extern int usb_driver_claim_interface(struct usb_driver *driver,
			struct usb_interface *iface, void* priv);

static inline int usb_interface_claimed(struct usb_interface *iface) {
	return (iface->dev.driver != NULL);
}

extern void usb_driver_release_interface(struct usb_driver *driver,
			struct usb_interface *iface);
const struct usb_device_id *usb_match_id(struct usb_interface *interface,
					 const struct usb_device_id *id);

extern struct usb_interface *usb_find_interface(struct usb_driver *drv,
		int minor);
extern struct usb_interface *usb_ifnum_to_if(struct usb_device *dev,
		unsigned ifnum);
extern struct usb_host_interface *usb_altnum_to_altsetting(
		struct usb_interface *intf, unsigned int altnum);


static inline int usb_make_path (struct usb_device *dev, char *buf,
		size_t size)
{
	int actual;
	actual = snprintf (buf, size, "usb-%s-%s", dev->bus->bus_name,
			dev->devpath);
	return (actual >= (int)size) ? -1 : actual;
}

/*-------------------------------------------------------------------------*/

#define USB_DEVICE_ID_MATCH_DEVICE \
		(USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_PRODUCT)
#define USB_DEVICE_ID_MATCH_DEV_RANGE \
		(USB_DEVICE_ID_MATCH_DEV_LO | USB_DEVICE_ID_MATCH_DEV_HI)
#define USB_DEVICE_ID_MATCH_DEVICE_AND_VERSION \
		(USB_DEVICE_ID_MATCH_DEVICE | USB_DEVICE_ID_MATCH_DEV_RANGE)
#define USB_DEVICE_ID_MATCH_DEV_INFO \
		(USB_DEVICE_ID_MATCH_DEV_CLASS | \
		USB_DEVICE_ID_MATCH_DEV_SUBCLASS | \
		USB_DEVICE_ID_MATCH_DEV_PROTOCOL)
#define USB_DEVICE_ID_MATCH_INT_INFO \
		(USB_DEVICE_ID_MATCH_INT_CLASS | \
		USB_DEVICE_ID_MATCH_INT_SUBCLASS | \
		USB_DEVICE_ID_MATCH_INT_PROTOCOL)

#define USB_DEVICE(vend,prod) \
	.match_flags = USB_DEVICE_ID_MATCH_DEVICE, .idVendor = (vend), \
			.idProduct = (prod)
#define USB_DEVICE_VER(vend,prod,lo,hi) \
	.match_flags = USB_DEVICE_ID_MATCH_DEVICE_AND_VERSION, \
	.idVendor = (vend), .idProduct = (prod), \
	.bcdDevice_lo = (lo), .bcdDevice_hi = (hi)

#define USB_DEVICE_INFO(cl,sc,pr) \
	.match_flags = USB_DEVICE_ID_MATCH_DEV_INFO, .bDeviceClass = (cl), \
	.bDeviceSubClass = (sc), .bDeviceProtocol = (pr)

#define USB_INTERFACE_INFO(cl,sc,pr) \
	.match_flags = USB_DEVICE_ID_MATCH_INT_INFO, .bInterfaceClass = (cl), \
	.bInterfaceSubClass = (sc), .bInterfaceProtocol = (pr)

/* ----------------------------------------------------------------------- */

struct usb_dynids {
	spinlock_t lock;
	struct list_head list;
};

struct usb_driver {
	const char *name;

	int (*probe) (struct usb_interface *intf,
		      const struct usb_device_id *id);

	void (*disconnect) (struct usb_interface *intf);

	int (*ioctl) (struct usb_interface *intf, unsigned int code,
			void *buf);

	int (*suspend) (struct usb_interface *intf, pm_message_t message);
	int (*resume) (struct usb_interface *intf);

	void (*pre_reset) (struct usb_interface *intf);
	void (*post_reset) (struct usb_interface *intf);

	const struct usb_device_id *id_table;

	struct usb_dynids dynids;
	struct device_driver driver;
	unsigned int no_dynamic_id:1;
};
#define	to_usb_driver(d) container_of(d, struct usb_driver, driver)

extern struct bus_type usb_bus_type;

struct usb_class_driver {
	char *name;
	const struct file_operations *fops;
	int minor_base;
};

int usb_register_driver(struct usb_driver *, struct module *);
static inline int usb_register(struct usb_driver *driver)
{
	return usb_register_driver(driver, THIS_MODULE);
}
extern void usb_deregister(struct usb_driver *);

extern int usb_register_dev(struct usb_interface *intf,
			    struct usb_class_driver *class_driver);
extern void usb_deregister_dev(struct usb_interface *intf,
			       struct usb_class_driver *class_driver);

extern int usb_disabled(void);

/* ----------------------------------------------------------------------- */


#define URB_SHORT_NOT_OK	0x0001	/* report short reads as errors */
#define URB_ISO_ASAP		0x0002	/* iso-only, urb->start_frame
					 * ignored */
#define URB_NO_TRANSFER_DMA_MAP	0x0004	/* urb->transfer_dma valid on submit */
#define URB_NO_SETUP_DMA_MAP	0x0008	/* urb->setup_dma valid on submit */
#define URB_NO_FSBR		0x0020	/* UHCI-specific */
#define URB_ZERO_PACKET		0x0040	/* Finish bulk OUT with short packet */
#define URB_NO_INTERRUPT	0x0080	/* HINT: no non-error interrupt
					 * needed */

struct usb_iso_packet_descriptor {
	unsigned int offset;
	unsigned int length;		/* expected length */
	unsigned int actual_length;
	unsigned int status;
};

struct urb;
struct pt_regs;

typedef void (*usb_complete_t)(struct urb *, struct pt_regs *);

struct urb
{
	/* private: usb core and host controller only fields in the urb */
	struct kref kref;		/* reference count of the URB */
	spinlock_t lock;		/* lock for the URB */
	void *hcpriv;			/* private data for host controller */
	int bandwidth;			/* bandwidth for INT/ISO request */
	atomic_t use_count;		/* concurrent submissions counter */
	u8 reject;			/* submissions will fail */

	/* public: documented fields in the urb that can be used by drivers */
	struct list_head urb_list;	/* list head for use by the urb's
					 * current owner */
	struct usb_device *dev; 	/* (in) pointer to associated device */
	unsigned int pipe;		/* (in) pipe information */
	int status;			/* (return) non-ISO status */
	unsigned int transfer_flags;	/* (in) URB_SHORT_NOT_OK | ...*/
	void *transfer_buffer;		/* (in) associated data buffer */
	dma_addr_t transfer_dma;	/* (in) dma addr for transfer_buffer */
	int transfer_buffer_length;	/* (in) data buffer length */
	int actual_length;		/* (return) actual transfer length */
	unsigned char *setup_packet;	/* (in) setup packet (control only) */
	dma_addr_t setup_dma;		/* (in) dma addr for setup_packet */
	int start_frame;		/* (modify) start frame (ISO) */
	int number_of_packets;		/* (in) number of ISO packets */
	int interval;			/* (modify) transfer interval
					 * (INT/ISO) */
	int error_count;		/* (return) number of ISO errors */
	void *context;			/* (in) context for completion */
	usb_complete_t complete;	/* (in) completion routine */
	struct usb_iso_packet_descriptor iso_frame_desc[0];
					/* (in) ISO ONLY */
};

/* ----------------------------------------------------------------------- */

static inline void usb_fill_control_urb (struct urb *urb,
					 struct usb_device *dev,
					 unsigned int pipe,
					 unsigned char *setup_packet,
					 void *transfer_buffer,
					 int buffer_length,
					 usb_complete_t complete,
					 void *context)
{
	spin_lock_init(&urb->lock);
	urb->dev = dev;
	urb->pipe = pipe;
	urb->setup_packet = setup_packet;
	urb->transfer_buffer = transfer_buffer;
	urb->transfer_buffer_length = buffer_length;
	urb->complete = complete;
	urb->context = context;
}

static inline void usb_fill_bulk_urb (struct urb *urb,
				      struct usb_device *dev,
				      unsigned int pipe,
				      void *transfer_buffer,
				      int buffer_length,
				      usb_complete_t complete,
				      void *context)
{
	spin_lock_init(&urb->lock);
	urb->dev = dev;
	urb->pipe = pipe;
	urb->transfer_buffer = transfer_buffer;
	urb->transfer_buffer_length = buffer_length;
	urb->complete = complete;
	urb->context = context;
}

static inline void usb_fill_int_urb (struct urb *urb,
				     struct usb_device *dev,
				     unsigned int pipe,
				     void *transfer_buffer,
				     int buffer_length,
				     usb_complete_t complete,
				     void *context,
				     int interval)
{
	spin_lock_init(&urb->lock);
	urb->dev = dev;
	urb->pipe = pipe;
	urb->transfer_buffer = transfer_buffer;
	urb->transfer_buffer_length = buffer_length;
	urb->complete = complete;
	urb->context = context;
	if (dev->speed == USB_SPEED_HIGH)
		urb->interval = 1 << (interval - 1);
	else
		urb->interval = interval;
	urb->start_frame = -1;
}

extern void usb_init_urb(struct urb *urb);
extern struct urb *usb_alloc_urb(int iso_packets, gfp_t mem_flags);
extern void usb_free_urb(struct urb *urb);
#define usb_put_urb usb_free_urb
extern struct urb *usb_get_urb(struct urb *urb);
extern int usb_submit_urb(struct urb *urb, gfp_t mem_flags);
extern int usb_unlink_urb(struct urb *urb);
extern void usb_kill_urb(struct urb *urb);

#define HAVE_USB_BUFFERS
void *usb_buffer_alloc (struct usb_device *dev, size_t size,
	gfp_t mem_flags, dma_addr_t *dma);
void usb_buffer_free (struct usb_device *dev, size_t size,
	void *addr, dma_addr_t dma);

#if 0
struct urb *usb_buffer_map (struct urb *urb);
void usb_buffer_dmasync (struct urb *urb);
void usb_buffer_unmap (struct urb *urb);
#endif

struct scatterlist;
int usb_buffer_map_sg (struct usb_device *dev, unsigned pipe,
		struct scatterlist *sg, int nents);
#if 0
void usb_buffer_dmasync_sg (struct usb_device *dev, unsigned pipe,
		struct scatterlist *sg, int n_hw_ents);
#endif
void usb_buffer_unmap_sg (struct usb_device *dev, unsigned pipe,
		struct scatterlist *sg, int n_hw_ents);


extern int usb_control_msg(struct usb_device *dev, unsigned int pipe,
	__u8 request, __u8 requesttype, __u16 value, __u16 index,
	void *data, __u16 size, int timeout);
extern int usb_interrupt_msg(struct usb_device *usb_dev, unsigned int pipe,
	void *data, int len, int *actual_length, int timeout);
extern int usb_bulk_msg(struct usb_device *usb_dev, unsigned int pipe,
	void *data, int len, int *actual_length,
	int timeout);

/* wrappers around usb_control_msg() for the most common standard requests */
extern int usb_get_descriptor(struct usb_device *dev, unsigned char desctype,
	unsigned char descindex, void *buf, int size);
extern int usb_get_status(struct usb_device *dev,
	int type, int target, void *data);
extern int usb_string(struct usb_device *dev, int index,
	char *buf, size_t size);

/* wrappers that also update important state inside usbcore */
extern int usb_clear_halt(struct usb_device *dev, int pipe);
extern int usb_reset_configuration(struct usb_device *dev);
extern int usb_set_interface(struct usb_device *dev, int ifnum, int alternate);

#define USB_CTRL_GET_TIMEOUT	5000
#define USB_CTRL_SET_TIMEOUT	5000


struct usb_sg_request {
	int			status;
	size_t			bytes;

	/* 
	 * members below are private: to usbcore,
	 * and are not provided for driver access!
	 */
	spinlock_t		lock;

	struct usb_device	*dev;
	int			pipe;
	struct scatterlist	*sg;
	int			nents;

	int			entries;
	struct urb		**urbs;

	int			count;
	struct completion	complete;
};

int usb_sg_init (
	struct usb_sg_request	*io,
	struct usb_device	*dev,
	unsigned		pipe, 
	unsigned		period,
	struct scatterlist	*sg,
	int			nents,
	size_t			length,
	gfp_t			mem_flags
);
void usb_sg_cancel (struct usb_sg_request *io);
void usb_sg_wait (struct usb_sg_request *io);


/* ----------------------------------------------------------------------- */


/* NOTE:  these are not the standard USB_ENDPOINT_XFER_* values!! */
/* (yet ... they're the values used by usbfs) */
#define PIPE_ISOCHRONOUS		0
#define PIPE_INTERRUPT			1
#define PIPE_CONTROL			2
#define PIPE_BULK			3

#define usb_pipein(pipe)	((pipe) & USB_DIR_IN)
#define usb_pipeout(pipe)	(!usb_pipein(pipe))

#define usb_pipedevice(pipe)	(((pipe) >> 8) & 0x7f)
#define usb_pipeendpoint(pipe)	(((pipe) >> 15) & 0xf)

#define usb_pipetype(pipe)	(((pipe) >> 30) & 3)
#define usb_pipeisoc(pipe)	(usb_pipetype((pipe)) == PIPE_ISOCHRONOUS)
#define usb_pipeint(pipe)	(usb_pipetype((pipe)) == PIPE_INTERRUPT)
#define usb_pipecontrol(pipe)	(usb_pipetype((pipe)) == PIPE_CONTROL)
#define usb_pipebulk(pipe)	(usb_pipetype((pipe)) == PIPE_BULK)

/* The D0/D1 toggle bits ... USE WITH CAUTION (they're almost hcd-internal) */
#define usb_gettoggle(dev, ep, out) (((dev)->toggle[out] >> (ep)) & 1)
#define	usb_dotoggle(dev, ep, out)  ((dev)->toggle[out] ^= (1 << (ep)))
#define usb_settoggle(dev, ep, out, bit) \
		((dev)->toggle[out] = ((dev)->toggle[out] & ~(1 << (ep))) | \
		 ((bit) << (ep)))


static inline unsigned int __create_pipe(struct usb_device *dev,
		unsigned int endpoint)
{
	return (dev->devnum << 8) | (endpoint << 15);
}

/* Create various pipes... */
#define usb_sndctrlpipe(dev,endpoint)	\
	((PIPE_CONTROL << 30) | __create_pipe(dev,endpoint))
#define usb_rcvctrlpipe(dev,endpoint)	\
	((PIPE_CONTROL << 30) | __create_pipe(dev,endpoint) | USB_DIR_IN)
#define usb_sndisocpipe(dev,endpoint)	\
	((PIPE_ISOCHRONOUS << 30) | __create_pipe(dev,endpoint))
#define usb_rcvisocpipe(dev,endpoint)	\
	((PIPE_ISOCHRONOUS << 30) | __create_pipe(dev,endpoint) | USB_DIR_IN)
#define usb_sndbulkpipe(dev,endpoint)	\
	((PIPE_BULK << 30) | __create_pipe(dev,endpoint))
#define usb_rcvbulkpipe(dev,endpoint)	\
	((PIPE_BULK << 30) | __create_pipe(dev,endpoint) | USB_DIR_IN)
#define usb_sndintpipe(dev,endpoint)	\
	((PIPE_INTERRUPT << 30) | __create_pipe(dev,endpoint))
#define usb_rcvintpipe(dev,endpoint)	\
	((PIPE_INTERRUPT << 30) | __create_pipe(dev,endpoint) | USB_DIR_IN)

/*-------------------------------------------------------------------------*/

static inline __u16
usb_maxpacket(struct usb_device *udev, int pipe, int is_out)
{
	struct usb_host_endpoint	*ep;
	unsigned			epnum = usb_pipeendpoint(pipe);

	if (is_out) {
		WARN_ON(usb_pipein(pipe));
		ep = udev->ep_out[epnum];
	} else {
		WARN_ON(usb_pipeout(pipe));
		ep = udev->ep_in[epnum];
	}
	if (!ep)
		return 0;

	/* NOTE:  only 0x07ff bits are for packet size... */
	return le16_to_cpu(ep->desc.wMaxPacketSize);
}

/* ----------------------------------------------------------------------- */

/* Events from the usb core */
#define USB_DEVICE_ADD		0x0001
#define USB_DEVICE_REMOVE	0x0002
#define USB_BUS_ADD		0x0003
#define USB_BUS_REMOVE		0x0004
extern void usb_register_notify(struct notifier_block *nb);
extern void usb_unregister_notify(struct notifier_block *nb);

#ifdef DEBUG
#define dbg(format, arg...) printk(KERN_DEBUG "%s: " format "\n" , \
	__FILE__ , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif

#define err(format, arg...) printk(KERN_ERR "%s: " format "\n" , \
	__FILE__ , ## arg)
#define info(format, arg...) printk(KERN_INFO "%s: " format "\n" , \
	__FILE__ , ## arg)
#define warn(format, arg...) printk(KERN_WARNING "%s: " format "\n" , \
	__FILE__ , ## arg)


#endif  /* __KERNEL__ */

#endif
