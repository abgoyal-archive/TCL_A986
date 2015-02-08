

#ifndef __HWAHC_INTERNAL_H__
#define __HWAHC_INTERNAL_H__

#include <linux/completion.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/uwb.h>
#include <linux/usb/wusb.h>
#include <linux/usb/wusb-wa.h>

struct wusbhc;
struct wahc;
extern void wa_urb_enqueue_run(struct work_struct *ws);

struct wa_rpipe {
	struct kref refcnt;
	struct usb_rpipe_descriptor descr;
	struct usb_host_endpoint *ep;
	struct wahc *wa;
	spinlock_t seg_lock;
	struct list_head seg_list;
	atomic_t segs_available;
	u8 buffer[1];	/* For reads/writes on USB */
};


struct wahc {
	struct usb_device *usb_dev;
	struct usb_interface *usb_iface;

	/* HC to deliver notifications */
	union {
		struct wusbhc *wusb;
		struct dwahc *dwa;
	};

	const struct usb_endpoint_descriptor *dto_epd, *dti_epd;
	const struct usb_wa_descriptor *wa_descr;

	struct urb *nep_urb;		/* Notification EndPoint [lockless] */
	struct edc nep_edc;
	void *nep_buffer;
	size_t nep_buffer_size;

	atomic_t notifs_queued;

	u16 rpipes;
	unsigned long *rpipe_bm;	/* rpipe usage bitmap */
	spinlock_t rpipe_bm_lock;	/* protect rpipe_bm */
	struct mutex rpipe_mutex;	/* assigning resources to endpoints */

	struct urb *dti_urb;		/* URB for reading xfer results */
	struct urb *buf_in_urb;		/* URB for reading data in */
	struct edc dti_edc;		/* DTI error density counter */
	struct wa_xfer_result *xfer_result; /* real size = dti_ep maxpktsize */
	size_t xfer_result_size;

	s32 status;			/* For reading status */

	struct list_head xfer_list;
	struct list_head xfer_delayed_list;
	spinlock_t xfer_list_lock;
	struct work_struct xfer_work;
	atomic_t xfer_id_count;
};


extern int wa_create(struct wahc *wa, struct usb_interface *iface);
extern void __wa_destroy(struct wahc *wa);
void wa_reset_all(struct wahc *wa);


/* Miscellaneous constants */
enum {
	/** Max number of EPROTO errors we tolerate on the NEP in a
	 * period of time */
	HWAHC_EPROTO_MAX = 16,
	/** Period of time for EPROTO errors (in jiffies) */
	HWAHC_EPROTO_PERIOD = 4 * HZ,
};


/* Notification endpoint handling */
extern int wa_nep_create(struct wahc *, struct usb_interface *);
extern void wa_nep_destroy(struct wahc *);

static inline int wa_nep_arm(struct wahc *wa, gfp_t gfp_mask)
{
	struct urb *urb = wa->nep_urb;
	urb->transfer_buffer = wa->nep_buffer;
	urb->transfer_buffer_length = wa->nep_buffer_size;
	return usb_submit_urb(urb, gfp_mask);
}

static inline void wa_nep_disarm(struct wahc *wa)
{
	usb_kill_urb(wa->nep_urb);
}


/* RPipes */
static inline void wa_rpipe_init(struct wahc *wa)
{
	spin_lock_init(&wa->rpipe_bm_lock);
	mutex_init(&wa->rpipe_mutex);
}

static inline void wa_init(struct wahc *wa)
{
	edc_init(&wa->nep_edc);
	atomic_set(&wa->notifs_queued, 0);
	wa_rpipe_init(wa);
	edc_init(&wa->dti_edc);
	INIT_LIST_HEAD(&wa->xfer_list);
	INIT_LIST_HEAD(&wa->xfer_delayed_list);
	spin_lock_init(&wa->xfer_list_lock);
	INIT_WORK(&wa->xfer_work, wa_urb_enqueue_run);
	atomic_set(&wa->xfer_id_count, 1);
}

struct wa_xfer;
extern void rpipe_destroy(struct kref *_rpipe);
static inline
void __rpipe_get(struct wa_rpipe *rpipe)
{
	kref_get(&rpipe->refcnt);
}
extern int rpipe_get_by_ep(struct wahc *, struct usb_host_endpoint *,
			   struct urb *, gfp_t);
static inline void rpipe_put(struct wa_rpipe *rpipe)
{
	kref_put(&rpipe->refcnt, rpipe_destroy);

}
extern void rpipe_ep_disable(struct wahc *, struct usb_host_endpoint *);
extern int wa_rpipes_create(struct wahc *);
extern void wa_rpipes_destroy(struct wahc *);
static inline void rpipe_avail_dec(struct wa_rpipe *rpipe)
{
	atomic_dec(&rpipe->segs_available);
}

static inline int rpipe_avail_inc(struct wa_rpipe *rpipe)
{
	return atomic_inc_return(&rpipe->segs_available) > 0
		&& !list_empty(&rpipe->seg_list);
}


/* Transferring data */
extern int wa_urb_enqueue(struct wahc *, struct usb_host_endpoint *,
			  struct urb *, gfp_t);
extern int wa_urb_dequeue(struct wahc *, struct urb *);
extern void wa_handle_notif_xfer(struct wahc *, struct wa_notif_hdr *);


static inline struct wahc *wa_get(struct wahc *wa)
{
	usb_get_intf(wa->usb_iface);
	return wa;
}

static inline void wa_put(struct wahc *wa)
{
	usb_put_intf(wa->usb_iface);
}


static inline int __wa_feature(struct wahc *wa, unsigned op, u16 feature)
{
	return usb_control_msg(wa->usb_dev, usb_sndctrlpipe(wa->usb_dev, 0),
			op ? USB_REQ_SET_FEATURE : USB_REQ_CLEAR_FEATURE,
			USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			feature,
			wa->usb_iface->cur_altsetting->desc.bInterfaceNumber,
			NULL, 0, 1000 /* FIXME: arbitrary */);
}


static inline int __wa_set_feature(struct wahc *wa, u16 feature)
{
	return  __wa_feature(wa, 1, feature);
}


static inline int __wa_clear_feature(struct wahc *wa, u16 feature)
{
	return __wa_feature(wa, 0, feature);
}


static inline
s32 __wa_get_status(struct wahc *wa)
{
	s32 result;
	result = usb_control_msg(
		wa->usb_dev, usb_rcvctrlpipe(wa->usb_dev, 0),
		USB_REQ_GET_STATUS,
		USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
		0, wa->usb_iface->cur_altsetting->desc.bInterfaceNumber,
		&wa->status, sizeof(wa->status),
		1000 /* FIXME: arbitrary */);
	if (result >= 0)
		result = wa->status;
	return result;
}


static inline s32 __wa_wait_status(struct wahc *wa, u32 mask, u32 value)
{
	s32 result;
	unsigned loops = 10;
	do {
		msleep(50);
		result = __wa_get_status(wa);
		if ((result & mask) == value)
			break;
		if (loops-- == 0) {
			result = -ETIMEDOUT;
			break;
		}
	} while (result >= 0);
	return result;
}


/** Command @hwahc to stop, @returns 0 if ok, < 0 errno code on error */
static inline int __wa_stop(struct wahc *wa)
{
	int result;
	struct device *dev = &wa->usb_iface->dev;

	result = __wa_clear_feature(wa, WA_ENABLE);
	if (result < 0 && result != -ENODEV) {
		dev_err(dev, "error commanding HC to stop: %d\n", result);
		goto out;
	}
	result = __wa_wait_status(wa, WA_ENABLE, 0);
	if (result < 0 && result != -ENODEV)
		dev_err(dev, "error waiting for HC to stop: %d\n", result);
out:
	return 0;
}


#endif /* #ifndef __HWAHC_INTERNAL_H__ */
