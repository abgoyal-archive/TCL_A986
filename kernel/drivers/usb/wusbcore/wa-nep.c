
#include <linux/workqueue.h>
#include <linux/ctype.h>
#include <linux/slab.h>

#include "wa-hc.h"
#include "wusbhc.h"

/* Structure for queueing notifications to the workqueue */
struct wa_notif_work {
	struct work_struct work;
	struct wahc *wa;
	size_t size;
	u8 data[];
};

static void wa_notif_dispatch(struct work_struct *ws)
{
	void *itr;
	u8 missing = 0;
	struct wa_notif_work *nw = container_of(ws, struct wa_notif_work, work);
	struct wahc *wa = nw->wa;
	struct wa_notif_hdr *notif_hdr;
	size_t size;

	struct device *dev = &wa->usb_iface->dev;

#if 0
	/* FIXME: need to check for this??? */
	if (usb_hcd->state == HC_STATE_QUIESCING)	/* Going down? */
		goto out;				/* screw it */
#endif
	atomic_dec(&wa->notifs_queued);		/* Throttling ctl */
	dev = &wa->usb_iface->dev;
	size = nw->size;
	itr = nw->data;

	while (size) {
		if (size < sizeof(*notif_hdr)) {
			missing = sizeof(*notif_hdr) - size;
			goto exhausted_buffer;
		}
		notif_hdr = itr;
		if (size < notif_hdr->bLength)
			goto exhausted_buffer;
		itr += notif_hdr->bLength;
		size -= notif_hdr->bLength;
		/* Dispatch the notification [don't use itr or size!] */
		switch (notif_hdr->bNotifyType) {
		case HWA_NOTIF_DN: {
			struct hwa_notif_dn *hwa_dn;
			hwa_dn = container_of(notif_hdr, struct hwa_notif_dn,
					      hdr);
			wusbhc_handle_dn(wa->wusb, hwa_dn->bSourceDeviceAddr,
					 hwa_dn->dndata,
					 notif_hdr->bLength - sizeof(*hwa_dn));
			break;
		}
		case WA_NOTIF_TRANSFER:
			wa_handle_notif_xfer(wa, notif_hdr);
			break;
		case DWA_NOTIF_RWAKE:
		case DWA_NOTIF_PORTSTATUS:
		case HWA_NOTIF_BPST_ADJ:
			/* FIXME: unimplemented WA NOTIFs */
			/* fallthru */
		default:
			dev_err(dev, "HWA: unknown notification 0x%x, "
				"%zu bytes; discarding\n",
				notif_hdr->bNotifyType,
				(size_t)notif_hdr->bLength);
			break;
		}
	}
out:
	wa_put(wa);
	kfree(nw);
	return;

	/* THIS SHOULD NOT HAPPEN
	 *
	 * Buffer exahusted with partial data remaining; just warn and
	 * discard the data, as this should not happen.
	 */
exhausted_buffer:
	dev_warn(dev, "HWA: device sent short notification, "
		 "%d bytes missing; discarding %d bytes.\n",
		 missing, (int)size);
	goto out;
}

static int wa_nep_queue(struct wahc *wa, size_t size)
{
	int result = 0;
	struct device *dev = &wa->usb_iface->dev;
	struct wa_notif_work *nw;

	/* dev_fnstart(dev, "(wa %p, size %zu)\n", wa, size); */
	BUG_ON(size > wa->nep_buffer_size);
	if (size == 0)
		goto out;
	if (atomic_read(&wa->notifs_queued) > 200) {
		if (printk_ratelimit())
			dev_err(dev, "Too many notifications queued, "
				"throttling back\n");
		goto out;
	}
	nw = kzalloc(sizeof(*nw) + size, GFP_ATOMIC);
	if (nw == NULL) {
		if (printk_ratelimit())
			dev_err(dev, "No memory to queue notification\n");
		goto out;
	}
	INIT_WORK(&nw->work, wa_notif_dispatch);
	nw->wa = wa_get(wa);
	nw->size = size;
	memcpy(nw->data, wa->nep_buffer, size);
	atomic_inc(&wa->notifs_queued);		/* Throttling ctl */
	queue_work(wusbd, &nw->work);
out:
	/* dev_fnend(dev, "(wa %p, size %zu) = result\n", wa, size, result); */
	return result;
}

static void wa_nep_cb(struct urb *urb)
{
	int result;
	struct wahc *wa = urb->context;
	struct device *dev = &wa->usb_iface->dev;

	switch (result = urb->status) {
	case 0:
		result = wa_nep_queue(wa, urb->actual_length);
		if (result < 0)
			dev_err(dev, "NEP: unable to process notification(s): "
				"%d\n", result);
		break;
	case -ECONNRESET:	/* Not an error, but a controlled situation; */
	case -ENOENT:		/* (we killed the URB)...so, no broadcast */
	case -ESHUTDOWN:
		dev_dbg(dev, "NEP: going down %d\n", urb->status);
		goto out;
	default:	/* On general errors, we retry unless it gets ugly */
		if (edc_inc(&wa->nep_edc, EDC_MAX_ERRORS,
			    EDC_ERROR_TIMEFRAME)) {
			dev_err(dev, "NEP: URB max acceptable errors "
				"exceeded, resetting device\n");
			wa_reset_all(wa);
			goto out;
		}
		dev_err(dev, "NEP: URB error %d\n", urb->status);
	}
	result = wa_nep_arm(wa, GFP_ATOMIC);
	if (result < 0) {
		dev_err(dev, "NEP: cannot submit URB: %d\n", result);
		wa_reset_all(wa);
	}
out:
	return;
}

int wa_nep_create(struct wahc *wa, struct usb_interface *iface)
{
	int result;
	struct usb_endpoint_descriptor *epd;
	struct usb_device *usb_dev = interface_to_usbdev(iface);
	struct device *dev = &iface->dev;

	edc_init(&wa->nep_edc);
	epd = &iface->cur_altsetting->endpoint[0].desc;
	wa->nep_buffer_size = 1024;
	wa->nep_buffer = kmalloc(wa->nep_buffer_size, GFP_KERNEL);
	if (wa->nep_buffer == NULL) {
		dev_err(dev, "Unable to allocate notification's read buffer\n");
		goto error_nep_buffer;
	}
	wa->nep_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (wa->nep_urb == NULL) {
		dev_err(dev, "Unable to allocate notification URB\n");
		goto error_urb_alloc;
	}
	usb_fill_int_urb(wa->nep_urb, usb_dev,
			 usb_rcvintpipe(usb_dev, epd->bEndpointAddress),
			 wa->nep_buffer, wa->nep_buffer_size,
			 wa_nep_cb, wa, epd->bInterval);
	result = wa_nep_arm(wa, GFP_KERNEL);
	if (result < 0) {
		dev_err(dev, "Cannot submit notification URB: %d\n", result);
		goto error_nep_arm;
	}
	return 0;

error_nep_arm:
	usb_free_urb(wa->nep_urb);
error_urb_alloc:
	kfree(wa->nep_buffer);
error_nep_buffer:
	return -ENOMEM;
}

void wa_nep_destroy(struct wahc *wa)
{
	wa_nep_disarm(wa);
	usb_free_urb(wa->nep_urb);
	kfree(wa->nep_buffer);
}
