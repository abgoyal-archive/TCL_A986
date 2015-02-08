

#include <linux/pci.h>	/* for scatterlist macros */
#include <linux/usb.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/timer.h>
#include <linux/ctype.h>
#include <linux/nls.h>
#include <linux/device.h>
#include <linux/scatterlist.h>
#include <linux/usb/quirks.h>
#include <linux/usb/hcd.h>	/* for usbcore internals */
#include <asm/byteorder.h>

#include "usb.h"

static void cancel_async_set_config(struct usb_device *udev);

struct api_context {
	struct completion	done;
	int			status;
};

static void usb_api_blocking_completion(struct urb *urb)
{
	struct api_context *ctx = urb->context;

	ctx->status = urb->status;
	complete(&ctx->done);
}


static int usb_start_wait_urb(struct urb *urb, int timeout, int *actual_length)
{
	struct api_context ctx;
	unsigned long expire;
	int retval;

	init_completion(&ctx.done);
	urb->context = &ctx;
	urb->actual_length = 0;
	retval = usb_submit_urb(urb, GFP_NOIO);
	if (unlikely(retval))
		goto out;

	expire = timeout ? msecs_to_jiffies(timeout) : MAX_SCHEDULE_TIMEOUT;
	if (!wait_for_completion_timeout(&ctx.done, expire)) {
		usb_kill_urb(urb);
		retval = (ctx.status == -ENOENT ? -ETIMEDOUT : ctx.status);

		dev_dbg(&urb->dev->dev,
			"%s timed out on ep%d%s len=%u/%u\n",
			current->comm,
			usb_endpoint_num(&urb->ep->desc),
			usb_urb_dir_in(urb) ? "in" : "out",
			urb->actual_length,
			urb->transfer_buffer_length);
	} else
		retval = ctx.status;
out:
	if (actual_length)
		*actual_length = urb->actual_length;

	usb_free_urb(urb);
	return retval;
}

/*-------------------------------------------------------------------*/
/* returns status (negative) or length (positive) */
static int usb_internal_control_msg(struct usb_device *usb_dev,
				    unsigned int pipe,
				    struct usb_ctrlrequest *cmd,
				    void *data, int len, int timeout)
{
	struct urb *urb;
	int retv;
	int length;

	urb = usb_alloc_urb(0, GFP_NOIO);
	if (!urb)
		return -ENOMEM;

	usb_fill_control_urb(urb, usb_dev, pipe, (unsigned char *)cmd, data,
			     len, usb_api_blocking_completion, NULL);

	retv = usb_start_wait_urb(urb, timeout, &length);
	if (retv < 0)
		return retv;
	else
		return length;
}

int usb_control_msg(struct usb_device *dev, unsigned int pipe, __u8 request,
		    __u8 requesttype, __u16 value, __u16 index, void *data,
		    __u16 size, int timeout)
{
	struct usb_ctrlrequest *dr;
	int ret;

	dr = kmalloc(sizeof(struct usb_ctrlrequest), GFP_NOIO);
	if (!dr)
		return -ENOMEM;

	dr->bRequestType = requesttype;
	dr->bRequest = request;
	dr->wValue = cpu_to_le16(value);
	dr->wIndex = cpu_to_le16(index);
	dr->wLength = cpu_to_le16(size);

	/* dbg("usb_control_msg"); */

	ret = usb_internal_control_msg(dev, pipe, dr, data, size, timeout);

	kfree(dr);

	return ret;
}
EXPORT_SYMBOL_GPL(usb_control_msg);

int usb_interrupt_msg(struct usb_device *usb_dev, unsigned int pipe,
		      void *data, int len, int *actual_length, int timeout)
{
	return usb_bulk_msg(usb_dev, pipe, data, len, actual_length, timeout);
}
EXPORT_SYMBOL_GPL(usb_interrupt_msg);

int usb_bulk_msg(struct usb_device *usb_dev, unsigned int pipe,
		 void *data, int len, int *actual_length, int timeout)
{
	struct urb *urb;
	struct usb_host_endpoint *ep;

	ep = usb_pipe_endpoint(usb_dev, pipe);
	if (!ep || len < 0)
		return -EINVAL;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
		return -ENOMEM;

	if ((ep->desc.bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
			USB_ENDPOINT_XFER_INT) {
		pipe = (pipe & ~(3 << 30)) | (PIPE_INTERRUPT << 30);
		usb_fill_int_urb(urb, usb_dev, pipe, data, len,
				usb_api_blocking_completion, NULL,
				ep->desc.bInterval);
	} else
		usb_fill_bulk_urb(urb, usb_dev, pipe, data, len,
				usb_api_blocking_completion, NULL);

	return usb_start_wait_urb(urb, timeout, actual_length);
}
EXPORT_SYMBOL_GPL(usb_bulk_msg);

/*-------------------------------------------------------------------*/

static void sg_clean(struct usb_sg_request *io)
{
	if (io->urbs) {
		while (io->entries--)
			usb_free_urb(io->urbs [io->entries]);
		kfree(io->urbs);
		io->urbs = NULL;
	}
	io->dev = NULL;
}

static void sg_complete(struct urb *urb)
{
	struct usb_sg_request *io = urb->context;
	int status = urb->status;

	spin_lock(&io->lock);

	/* In 2.5 we require hcds' endpoint queues not to progress after fault
	 * reports, until the completion callback (this!) returns.  That lets
	 * device driver code (like this routine) unlink queued urbs first,
	 * if it needs to, since the HC won't work on them at all.  So it's
	 * not possible for page N+1 to overwrite page N, and so on.
	 *
	 * That's only for "hard" faults; "soft" faults (unlinks) sometimes
	 * complete before the HCD can get requests away from hardware,
	 * though never during cleanup after a hard fault.
	 */
	if (io->status
			&& (io->status != -ECONNRESET
				|| status != -ECONNRESET)
			&& urb->actual_length) {
		dev_err(io->dev->bus->controller,
			"dev %s ep%d%s scatterlist error %d/%d\n",
			io->dev->devpath,
			usb_endpoint_num(&urb->ep->desc),
			usb_urb_dir_in(urb) ? "in" : "out",
			status, io->status);
		/* BUG (); */
	}

	if (io->status == 0 && status && status != -ECONNRESET) {
		int i, found, retval;

		io->status = status;

		/* the previous urbs, and this one, completed already.
		 * unlink pending urbs so they won't rx/tx bad data.
		 * careful: unlink can sometimes be synchronous...
		 */
		spin_unlock(&io->lock);
		for (i = 0, found = 0; i < io->entries; i++) {
			if (!io->urbs [i] || !io->urbs [i]->dev)
				continue;
			if (found) {
				retval = usb_unlink_urb(io->urbs [i]);
				if (retval != -EINPROGRESS &&
				    retval != -ENODEV &&
				    retval != -EBUSY)
					dev_err(&io->dev->dev,
						"%s, unlink --> %d\n",
						__func__, retval);
			} else if (urb == io->urbs [i])
				found = 1;
		}
		spin_lock(&io->lock);
	}
	urb->dev = NULL;

	/* on the last completion, signal usb_sg_wait() */
	io->bytes += urb->actual_length;
	io->count--;
	if (!io->count)
		complete(&io->complete);

	spin_unlock(&io->lock);
}


int usb_sg_init(struct usb_sg_request *io, struct usb_device *dev,
		unsigned pipe, unsigned	period, struct scatterlist *sg,
		int nents, size_t length, gfp_t mem_flags)
{
	int i;
	int urb_flags;
	int use_sg;

	if (!io || !dev || !sg
			|| usb_pipecontrol(pipe)
			|| usb_pipeisoc(pipe)
			|| nents <= 0)
		return -EINVAL;

	spin_lock_init(&io->lock);
	io->dev = dev;
	io->pipe = pipe;

	if (dev->bus->sg_tablesize > 0) {
		use_sg = true;
		io->entries = 1;
	} else {
		use_sg = false;
		io->entries = nents;
	}

	/* initialize all the urbs we'll use */
	io->urbs = kmalloc(io->entries * sizeof *io->urbs, mem_flags);
	if (!io->urbs)
		goto nomem;

	urb_flags = URB_NO_INTERRUPT;
	if (usb_pipein(pipe))
		urb_flags |= URB_SHORT_NOT_OK;

	for_each_sg(sg, sg, io->entries, i) {
		struct urb *urb;
		unsigned len;

		urb = usb_alloc_urb(0, mem_flags);
		if (!urb) {
			io->entries = i;
			goto nomem;
		}
		io->urbs[i] = urb;

		urb->dev = NULL;
		urb->pipe = pipe;
		urb->interval = period;
		urb->transfer_flags = urb_flags;
		urb->complete = sg_complete;
		urb->context = io;
		urb->sg = sg;

		if (use_sg) {
			/* There is no single transfer buffer */
			urb->transfer_buffer = NULL;
			urb->num_sgs = nents;

			/* A length of zero means transfer the whole sg list */
			len = length;
			if (len == 0) {
				struct scatterlist	*sg2;
				int			j;

				for_each_sg(sg, sg2, nents, j)
					len += sg2->length;
			}
		} else {
			/*
			 * Some systems can't use DMA; they use PIO instead.
			 * For their sakes, transfer_buffer is set whenever
			 * possible.
			 */
			if (!PageHighMem(sg_page(sg)))
				urb->transfer_buffer = sg_virt(sg);
			else
				urb->transfer_buffer = NULL;

			len = sg->length;
			if (length) {
				len = min_t(unsigned, len, length);
				length -= len;
				if (length == 0)
					io->entries = i + 1;
			}
		}
		urb->transfer_buffer_length = len;
	}
	io->urbs[--i]->transfer_flags &= ~URB_NO_INTERRUPT;

	/* transaction state */
	io->count = io->entries;
	io->status = 0;
	io->bytes = 0;
	init_completion(&io->complete);
	return 0;

nomem:
	sg_clean(io);
	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(usb_sg_init);

void usb_sg_wait(struct usb_sg_request *io)
{
	int i;
	int entries = io->entries;

	/* queue the urbs.  */
	spin_lock_irq(&io->lock);
	i = 0;
	while (i < entries && !io->status) {
		int retval;

		io->urbs[i]->dev = io->dev;
		retval = usb_submit_urb(io->urbs [i], GFP_ATOMIC);

		/* after we submit, let completions or cancelations fire;
		 * we handshake using io->status.
		 */
		spin_unlock_irq(&io->lock);
		switch (retval) {
			/* maybe we retrying will recover */
		case -ENXIO:	/* hc didn't queue this one */
		case -EAGAIN:
		case -ENOMEM:
			io->urbs[i]->dev = NULL;
			retval = 0;
			yield();
			break;

			/* no error? continue immediately.
			 *
			 * NOTE: to work better with UHCI (4K I/O buffer may
			 * need 3K of TDs) it may be good to limit how many
			 * URBs are queued at once; N milliseconds?
			 */
		case 0:
			++i;
			cpu_relax();
			break;

			/* fail any uncompleted urbs */
		default:
			io->urbs[i]->dev = NULL;
			io->urbs[i]->status = retval;
			dev_dbg(&io->dev->dev, "%s, submit --> %d\n",
				__func__, retval);
			usb_sg_cancel(io);
		}
		spin_lock_irq(&io->lock);
		if (retval && (io->status == 0 || io->status == -ECONNRESET))
			io->status = retval;
	}
	io->count -= entries - i;
	if (io->count == 0)
		complete(&io->complete);
	spin_unlock_irq(&io->lock);

	/* OK, yes, this could be packaged as non-blocking.
	 * So could the submit loop above ... but it's easier to
	 * solve neither problem than to solve both!
	 */
	wait_for_completion(&io->complete);

	sg_clean(io);
}
EXPORT_SYMBOL_GPL(usb_sg_wait);

void usb_sg_cancel(struct usb_sg_request *io)
{
	unsigned long flags;

	spin_lock_irqsave(&io->lock, flags);

	/* shut everything down, if it didn't already */
	if (!io->status) {
		int i;

		io->status = -ECONNRESET;
		spin_unlock(&io->lock);
		for (i = 0; i < io->entries; i++) {
			int retval;

			if (!io->urbs [i]->dev)
				continue;
			retval = usb_unlink_urb(io->urbs [i]);
			if (retval != -EINPROGRESS && retval != -EBUSY)
				dev_warn(&io->dev->dev, "%s, unlink --> %d\n",
					__func__, retval);
		}
		spin_lock(&io->lock);
	}
	spin_unlock_irqrestore(&io->lock, flags);
}
EXPORT_SYMBOL_GPL(usb_sg_cancel);

/*-------------------------------------------------------------------*/

int usb_get_descriptor(struct usb_device *dev, unsigned char type,
		       unsigned char index, void *buf, int size)
{
	int i;
	int result;

	memset(buf, 0, size);	/* Make sure we parse really received data */

	for (i = 0; i < 3; ++i) {
		/* retry on length 0 or error; some devices are flakey */
		result = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
				USB_REQ_GET_DESCRIPTOR, USB_DIR_IN,
				(type << 8) + index, 0, buf, size,
				USB_CTRL_GET_TIMEOUT);
		if (result <= 0 && result != -ETIMEDOUT)
			continue;
		if (result > 1 && ((u8 *)buf)[1] != type) {
			result = -ENODATA;
			continue;
		}
		break;
	}
	return result;
}
EXPORT_SYMBOL_GPL(usb_get_descriptor);

static int usb_get_string(struct usb_device *dev, unsigned short langid,
			  unsigned char index, void *buf, int size)
{
	int i;
	int result;

	for (i = 0; i < 3; ++i) {
		/* retry on length 0 or stall; some devices are flakey */
		result = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
			USB_REQ_GET_DESCRIPTOR, USB_DIR_IN,
			(USB_DT_STRING << 8) + index, langid, buf, size,
			USB_CTRL_GET_TIMEOUT);
		if (result == 0 || result == -EPIPE)
			continue;
		if (result > 1 && ((u8 *) buf)[1] != USB_DT_STRING) {
			result = -ENODATA;
			continue;
		}
		break;
	}
	return result;
}

static void usb_try_string_workarounds(unsigned char *buf, int *length)
{
	int newlength, oldlength = *length;

	for (newlength = 2; newlength + 1 < oldlength; newlength += 2)
		if (!isprint(buf[newlength]) || buf[newlength + 1])
			break;

	if (newlength > 2) {
		buf[0] = newlength;
		*length = newlength;
	}
}

static int usb_string_sub(struct usb_device *dev, unsigned int langid,
			  unsigned int index, unsigned char *buf)
{
	int rc;

	/* Try to read the string descriptor by asking for the maximum
	 * possible number of bytes */
	if (dev->quirks & USB_QUIRK_STRING_FETCH_255)
		rc = -EIO;
	else
		rc = usb_get_string(dev, langid, index, buf, 255);

	/* If that failed try to read the descriptor length, then
	 * ask for just that many bytes */
	if (rc < 2) {
		rc = usb_get_string(dev, langid, index, buf, 2);
		if (rc == 2)
			rc = usb_get_string(dev, langid, index, buf, buf[0]);
	}

	if (rc >= 2) {
		if (!buf[0] && !buf[1])
			usb_try_string_workarounds(buf, &rc);

		/* There might be extra junk at the end of the descriptor */
		if (buf[0] < rc)
			rc = buf[0];

		rc = rc - (rc & 1); /* force a multiple of two */
	}

	if (rc < 2)
		rc = (rc < 0 ? rc : -EINVAL);

	return rc;
}

static int usb_get_langid(struct usb_device *dev, unsigned char *tbuf)
{
	int err;

	if (dev->have_langid)
		return 0;

	if (dev->string_langid < 0)
		return -EPIPE;

	err = usb_string_sub(dev, 0, 0, tbuf);

	/* If the string was reported but is malformed, default to english
	 * (0x0409) */
	if (err == -ENODATA || (err > 0 && err < 4)) {
		dev->string_langid = 0x0409;
		dev->have_langid = 1;
		dev_err(&dev->dev,
			"string descriptor 0 malformed (err = %d), "
			"defaulting to 0x%04x\n",
				err, dev->string_langid);
		return 0;
	}

	/* In case of all other errors, we assume the device is not able to
	 * deal with strings at all. Set string_langid to -1 in order to
	 * prevent any string to be retrieved from the device */
	if (err < 0) {
		dev_err(&dev->dev, "string descriptor 0 read error: %d\n",
					err);
		dev->string_langid = -1;
		return -EPIPE;
	}

	/* always use the first langid listed */
	dev->string_langid = tbuf[2] | (tbuf[3] << 8);
	dev->have_langid = 1;
	dev_dbg(&dev->dev, "default language 0x%04x\n",
				dev->string_langid);
	return 0;
}

int usb_string(struct usb_device *dev, int index, char *buf, size_t size)
{
	unsigned char *tbuf;
	int err;

	if (dev->state == USB_STATE_SUSPENDED)
		return -EHOSTUNREACH;
	if (size <= 0 || !buf || !index)
		return -EINVAL;
	buf[0] = 0;
	tbuf = kmalloc(256, GFP_NOIO);
	if (!tbuf)
		return -ENOMEM;

	err = usb_get_langid(dev, tbuf);
	if (err < 0)
		goto errout;

	err = usb_string_sub(dev, dev->string_langid, index, tbuf);
	if (err < 0)
		goto errout;

	size--;		/* leave room for trailing NULL char in output buffer */
	err = utf16s_to_utf8s((wchar_t *) &tbuf[2], (err - 2) / 2,
			UTF16_LITTLE_ENDIAN, buf, size);
	buf[err] = 0;

	if (tbuf[1] != USB_DT_STRING)
		dev_dbg(&dev->dev,
			"wrong descriptor type %02x for string %d (\"%s\")\n",
			tbuf[1], index, buf);

 errout:
	kfree(tbuf);
	return err;
}
EXPORT_SYMBOL_GPL(usb_string);

/* one UTF-8-encoded 16-bit character has at most three bytes */
#define MAX_USB_STRING_SIZE (127 * 3 + 1)

char *usb_cache_string(struct usb_device *udev, int index)
{
	char *buf;
	char *smallbuf = NULL;
	int len;

	if (index <= 0)
		return NULL;

	buf = kmalloc(MAX_USB_STRING_SIZE, GFP_NOIO);
	if (buf) {
		len = usb_string(udev, index, buf, MAX_USB_STRING_SIZE);
		if (len > 0) {
			smallbuf = kmalloc(++len, GFP_NOIO);
			if (!smallbuf)
				return buf;
			memcpy(smallbuf, buf, len);
		}
		kfree(buf);
	}
	return smallbuf;
}

int usb_get_device_descriptor(struct usb_device *dev, unsigned int size)
{
	struct usb_device_descriptor *desc;
	int ret;

	if (size > sizeof(*desc))
		return -EINVAL;
	desc = kmalloc(sizeof(*desc), GFP_NOIO);
	if (!desc)
		return -ENOMEM;

	ret = usb_get_descriptor(dev, USB_DT_DEVICE, 0, desc, size);
	if (ret >= 0)
		memcpy(&dev->descriptor, desc, size);
	kfree(desc);
	return ret;
}

int usb_get_status(struct usb_device *dev, int type, int target, void *data)
{
	int ret;
	u16 *status = kmalloc(sizeof(*status), GFP_KERNEL);

	if (!status)
		return -ENOMEM;

	ret = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
		USB_REQ_GET_STATUS, USB_DIR_IN | type, 0, target, status,
		sizeof(*status), USB_CTRL_GET_TIMEOUT);

	*(u16 *)data = *status;
	kfree(status);
	return ret;
}
EXPORT_SYMBOL_GPL(usb_get_status);

int usb_clear_halt(struct usb_device *dev, int pipe)
{
	int result;
	int endp = usb_pipeendpoint(pipe);

	if (usb_pipein(pipe))
		endp |= USB_DIR_IN;

	/* we don't care if it wasn't halted first. in fact some devices
	 * (like some ibmcam model 1 units) seem to expect hosts to make
	 * this request for iso endpoints, which can't halt!
	 */
	result = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
		USB_REQ_CLEAR_FEATURE, USB_RECIP_ENDPOINT,
		USB_ENDPOINT_HALT, endp, NULL, 0,
		USB_CTRL_SET_TIMEOUT);

	/* don't un-halt or force to DATA0 except on success */
	if (result < 0)
		return result;

	/* NOTE:  seems like Microsoft and Apple don't bother verifying
	 * the clear "took", so some devices could lock up if you check...
	 * such as the Hagiwara FlashGate DUAL.  So we won't bother.
	 *
	 * NOTE:  make sure the logic here doesn't diverge much from
	 * the copy in usb-storage, for as long as we need two copies.
	 */

	usb_reset_endpoint(dev, endp);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_clear_halt);

static int create_intf_ep_devs(struct usb_interface *intf)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_host_interface *alt = intf->cur_altsetting;
	int i;

	if (intf->ep_devs_created || intf->unregistering)
		return 0;

	for (i = 0; i < alt->desc.bNumEndpoints; ++i)
		(void) usb_create_ep_devs(&intf->dev, &alt->endpoint[i], udev);
	intf->ep_devs_created = 1;
	return 0;
}

static void remove_intf_ep_devs(struct usb_interface *intf)
{
	struct usb_host_interface *alt = intf->cur_altsetting;
	int i;

	if (!intf->ep_devs_created)
		return;

	for (i = 0; i < alt->desc.bNumEndpoints; ++i)
		usb_remove_ep_devs(&alt->endpoint[i]);
	intf->ep_devs_created = 0;
}

void usb_disable_endpoint(struct usb_device *dev, unsigned int epaddr,
		bool reset_hardware)
{
	unsigned int epnum = epaddr & USB_ENDPOINT_NUMBER_MASK;
	struct usb_host_endpoint *ep;

	if (!dev)
		return;

	if (usb_endpoint_out(epaddr)) {
		ep = dev->ep_out[epnum];
		if (reset_hardware)
			dev->ep_out[epnum] = NULL;
	} else {
		ep = dev->ep_in[epnum];
		if (reset_hardware)
			dev->ep_in[epnum] = NULL;
	}
	if (ep) {
		ep->enabled = 0;
		usb_hcd_flush_endpoint(dev, ep);
		if (reset_hardware)
			usb_hcd_disable_endpoint(dev, ep);
	}
}

void usb_reset_endpoint(struct usb_device *dev, unsigned int epaddr)
{
	unsigned int epnum = epaddr & USB_ENDPOINT_NUMBER_MASK;
	struct usb_host_endpoint *ep;

	if (usb_endpoint_out(epaddr))
		ep = dev->ep_out[epnum];
	else
		ep = dev->ep_in[epnum];
	if (ep)
		usb_hcd_reset_endpoint(dev, ep);
}
EXPORT_SYMBOL_GPL(usb_reset_endpoint);


void usb_disable_interface(struct usb_device *dev, struct usb_interface *intf,
		bool reset_hardware)
{
	struct usb_host_interface *alt = intf->cur_altsetting;
	int i;

	for (i = 0; i < alt->desc.bNumEndpoints; ++i) {
		usb_disable_endpoint(dev,
				alt->endpoint[i].desc.bEndpointAddress,
				reset_hardware);
	}
}

void usb_disable_device(struct usb_device *dev, int skip_ep0)
{
	int i;

	dev_dbg(&dev->dev, "%s nuking %s URBs\n", __func__,
		skip_ep0 ? "non-ep0" : "all");
	for (i = skip_ep0; i < 16; ++i) {
		usb_disable_endpoint(dev, i, true);
		usb_disable_endpoint(dev, i + USB_DIR_IN, true);
	}

	/* getting rid of interfaces will disconnect
	 * any drivers bound to them (a key side effect)
	 */
	if (dev->actconfig) {
		for (i = 0; i < dev->actconfig->desc.bNumInterfaces; i++) {
			struct usb_interface	*interface;

			/* remove this interface if it has been registered */
			interface = dev->actconfig->interface[i];
			if (!device_is_registered(&interface->dev))
				continue;
			dev_dbg(&dev->dev, "unregistering interface %s\n",
				dev_name(&interface->dev));
			interface->unregistering = 1;
			remove_intf_ep_devs(interface);
			device_del(&interface->dev);
		}

		/* Now that the interfaces are unbound, nobody should
		 * try to access them.
		 */
		for (i = 0; i < dev->actconfig->desc.bNumInterfaces; i++) {
			put_device(&dev->actconfig->interface[i]->dev);
			dev->actconfig->interface[i] = NULL;
		}
		dev->actconfig = NULL;
		if (dev->state == USB_STATE_CONFIGURED)
			usb_set_device_state(dev, USB_STATE_ADDRESS);
	}
}

void usb_enable_endpoint(struct usb_device *dev, struct usb_host_endpoint *ep,
		bool reset_ep)
{
	int epnum = usb_endpoint_num(&ep->desc);
	int is_out = usb_endpoint_dir_out(&ep->desc);
	int is_control = usb_endpoint_xfer_control(&ep->desc);

	if (reset_ep)
		usb_hcd_reset_endpoint(dev, ep);
	if (is_out || is_control)
		dev->ep_out[epnum] = ep;
	if (!is_out || is_control)
		dev->ep_in[epnum] = ep;
	ep->enabled = 1;
}

void usb_enable_interface(struct usb_device *dev,
		struct usb_interface *intf, bool reset_eps)
{
	struct usb_host_interface *alt = intf->cur_altsetting;
	int i;

	for (i = 0; i < alt->desc.bNumEndpoints; ++i)
		usb_enable_endpoint(dev, &alt->endpoint[i], reset_eps);
}

int usb_set_interface(struct usb_device *dev, int interface, int alternate)
{
	struct usb_interface *iface;
	struct usb_host_interface *alt;
	struct usb_hcd *hcd = bus_to_hcd(dev->bus);
	int ret;
	int manual = 0;
	unsigned int epaddr;
	unsigned int pipe;

	if (dev->state == USB_STATE_SUSPENDED)
		return -EHOSTUNREACH;

	iface = usb_ifnum_to_if(dev, interface);
	if (!iface) {
		dev_dbg(&dev->dev, "selecting invalid interface %d\n",
			interface);
		return -EINVAL;
	}

	alt = usb_altnum_to_altsetting(iface, alternate);
	if (!alt) {
		dev_warn(&dev->dev, "selecting invalid altsetting %d\n",
			 alternate);
		return -EINVAL;
	}

	/* Make sure we have enough bandwidth for this alternate interface.
	 * Remove the current alt setting and add the new alt setting.
	 */
	mutex_lock(&hcd->bandwidth_mutex);
	ret = usb_hcd_alloc_bandwidth(dev, NULL, iface->cur_altsetting, alt);
	if (ret < 0) {
		dev_info(&dev->dev, "Not enough bandwidth for altsetting %d\n",
				alternate);
		mutex_unlock(&hcd->bandwidth_mutex);
		return ret;
	}

	if (dev->quirks & USB_QUIRK_NO_SET_INTF)
		ret = -EPIPE;
	else
		ret = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
				   USB_REQ_SET_INTERFACE, USB_RECIP_INTERFACE,
				   alternate, interface, NULL, 0, 5000);

	/* 9.4.10 says devices don't need this and are free to STALL the
	 * request if the interface only has one alternate setting.
	 */
	if (ret == -EPIPE && iface->num_altsetting == 1) {
		dev_dbg(&dev->dev,
			"manual set_interface for iface %d, alt %d\n",
			interface, alternate);
		manual = 1;
	} else if (ret < 0) {
		/* Re-instate the old alt setting */
		usb_hcd_alloc_bandwidth(dev, NULL, alt, iface->cur_altsetting);
		mutex_unlock(&hcd->bandwidth_mutex);
		return ret;
	}
	mutex_unlock(&hcd->bandwidth_mutex);

	/* FIXME drivers shouldn't need to replicate/bugfix the logic here
	 * when they implement async or easily-killable versions of this or
	 * other "should-be-internal" functions (like clear_halt).
	 * should hcd+usbcore postprocess control requests?
	 */

	/* prevent submissions using previous endpoint settings */
	if (iface->cur_altsetting != alt) {
		remove_intf_ep_devs(iface);
		usb_remove_sysfs_intf_files(iface);
	}
	usb_disable_interface(dev, iface, true);

	iface->cur_altsetting = alt;

	/* If the interface only has one altsetting and the device didn't
	 * accept the request, we attempt to carry out the equivalent action
	 * by manually clearing the HALT feature for each endpoint in the
	 * new altsetting.
	 */
	if (manual) {
		int i;

		for (i = 0; i < alt->desc.bNumEndpoints; i++) {
			epaddr = alt->endpoint[i].desc.bEndpointAddress;
			pipe = __create_pipe(dev,
					USB_ENDPOINT_NUMBER_MASK & epaddr) |
					(usb_endpoint_out(epaddr) ?
					USB_DIR_OUT : USB_DIR_IN);

			usb_clear_halt(dev, pipe);
		}
	}

	/* 9.1.1.5: reset toggles for all endpoints in the new altsetting
	 *
	 * Note:
	 * Despite EP0 is always present in all interfaces/AS, the list of
	 * endpoints from the descriptor does not contain EP0. Due to its
	 * omnipresence one might expect EP0 being considered "affected" by
	 * any SetInterface request and hence assume toggles need to be reset.
	 * However, EP0 toggles are re-synced for every individual transfer
	 * during the SETUP stage - hence EP0 toggles are "don't care" here.
	 * (Likewise, EP0 never "halts" on well designed devices.)
	 */
	usb_enable_interface(dev, iface, true);
	if (device_is_registered(&iface->dev)) {
		usb_create_sysfs_intf_files(iface);
		create_intf_ep_devs(iface);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(usb_set_interface);

int usb_reset_configuration(struct usb_device *dev)
{
	int			i, retval;
	struct usb_host_config	*config;
	struct usb_hcd *hcd = bus_to_hcd(dev->bus);

	if (dev->state == USB_STATE_SUSPENDED)
		return -EHOSTUNREACH;

	/* caller must have locked the device and must own
	 * the usb bus readlock (so driver bindings are stable);
	 * calls during probe() are fine
	 */

	for (i = 1; i < 16; ++i) {
		usb_disable_endpoint(dev, i, true);
		usb_disable_endpoint(dev, i + USB_DIR_IN, true);
	}

	config = dev->actconfig;
	retval = 0;
	mutex_lock(&hcd->bandwidth_mutex);
	/* Make sure we have enough bandwidth for each alternate setting 0 */
	for (i = 0; i < config->desc.bNumInterfaces; i++) {
		struct usb_interface *intf = config->interface[i];
		struct usb_host_interface *alt;

		alt = usb_altnum_to_altsetting(intf, 0);
		if (!alt)
			alt = &intf->altsetting[0];
		if (alt != intf->cur_altsetting)
			retval = usb_hcd_alloc_bandwidth(dev, NULL,
					intf->cur_altsetting, alt);
		if (retval < 0)
			break;
	}
	/* If not, reinstate the old alternate settings */
	if (retval < 0) {
reset_old_alts:
		for (i--; i >= 0; i--) {
			struct usb_interface *intf = config->interface[i];
			struct usb_host_interface *alt;

			alt = usb_altnum_to_altsetting(intf, 0);
			if (!alt)
				alt = &intf->altsetting[0];
			if (alt != intf->cur_altsetting)
				usb_hcd_alloc_bandwidth(dev, NULL,
						alt, intf->cur_altsetting);
		}
		mutex_unlock(&hcd->bandwidth_mutex);
		return retval;
	}
	retval = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
			USB_REQ_SET_CONFIGURATION, 0,
			config->desc.bConfigurationValue, 0,
			NULL, 0, USB_CTRL_SET_TIMEOUT);
	if (retval < 0)
		goto reset_old_alts;
	mutex_unlock(&hcd->bandwidth_mutex);

	/* re-init hc/hcd interface/endpoint state */
	for (i = 0; i < config->desc.bNumInterfaces; i++) {
		struct usb_interface *intf = config->interface[i];
		struct usb_host_interface *alt;

		alt = usb_altnum_to_altsetting(intf, 0);

		/* No altsetting 0?  We'll assume the first altsetting.
		 * We could use a GetInterface call, but if a device is
		 * so non-compliant that it doesn't have altsetting 0
		 * then I wouldn't trust its reply anyway.
		 */
		if (!alt)
			alt = &intf->altsetting[0];

		if (alt != intf->cur_altsetting) {
			remove_intf_ep_devs(intf);
			usb_remove_sysfs_intf_files(intf);
		}
		intf->cur_altsetting = alt;
		usb_enable_interface(dev, intf, true);
		if (device_is_registered(&intf->dev)) {
			usb_create_sysfs_intf_files(intf);
			create_intf_ep_devs(intf);
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(usb_reset_configuration);

static void usb_release_interface(struct device *dev)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct usb_interface_cache *intfc =
			altsetting_to_usb_interface_cache(intf->altsetting);

	kref_put(&intfc->ref, usb_release_interface_cache);
	kfree(intf);
}

#ifdef	CONFIG_HOTPLUG
static int usb_if_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct usb_device *usb_dev;
	struct usb_interface *intf;
	struct usb_host_interface *alt;

	intf = to_usb_interface(dev);
	usb_dev = interface_to_usbdev(intf);
	alt = intf->cur_altsetting;

	if (add_uevent_var(env, "INTERFACE=%d/%d/%d",
		   alt->desc.bInterfaceClass,
		   alt->desc.bInterfaceSubClass,
		   alt->desc.bInterfaceProtocol))
		return -ENOMEM;

	if (add_uevent_var(env,
		   "MODALIAS=usb:"
		   "v%04Xp%04Xd%04Xdc%02Xdsc%02Xdp%02Xic%02Xisc%02Xip%02X",
		   le16_to_cpu(usb_dev->descriptor.idVendor),
		   le16_to_cpu(usb_dev->descriptor.idProduct),
		   le16_to_cpu(usb_dev->descriptor.bcdDevice),
		   usb_dev->descriptor.bDeviceClass,
		   usb_dev->descriptor.bDeviceSubClass,
		   usb_dev->descriptor.bDeviceProtocol,
		   alt->desc.bInterfaceClass,
		   alt->desc.bInterfaceSubClass,
		   alt->desc.bInterfaceProtocol))
		return -ENOMEM;

	return 0;
}

#else

static int usb_if_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	return -ENODEV;
}
#endif	/* CONFIG_HOTPLUG */

struct device_type usb_if_device_type = {
	.name =		"usb_interface",
	.release =	usb_release_interface,
	.uevent =	usb_if_uevent,
};

static struct usb_interface_assoc_descriptor *find_iad(struct usb_device *dev,
						struct usb_host_config *config,
						u8 inum)
{
	struct usb_interface_assoc_descriptor *retval = NULL;
	struct usb_interface_assoc_descriptor *intf_assoc;
	int first_intf;
	int last_intf;
	int i;

	for (i = 0; (i < USB_MAXIADS && config->intf_assoc[i]); i++) {
		intf_assoc = config->intf_assoc[i];
		if (intf_assoc->bInterfaceCount == 0)
			continue;

		first_intf = intf_assoc->bFirstInterface;
		last_intf = first_intf + (intf_assoc->bInterfaceCount - 1);
		if (inum >= first_intf && inum <= last_intf) {
			if (!retval)
				retval = intf_assoc;
			else
				dev_err(&dev->dev, "Interface #%d referenced"
					" by multiple IADs\n", inum);
		}
	}

	return retval;
}


static void __usb_queue_reset_device(struct work_struct *ws)
{
	int rc;
	struct usb_interface *iface =
		container_of(ws, struct usb_interface, reset_ws);
	struct usb_device *udev = interface_to_usbdev(iface);

	rc = usb_lock_device_for_reset(udev, iface);
	if (rc >= 0) {
		iface->reset_running = 1;
		usb_reset_device(udev);
		iface->reset_running = 0;
		usb_unlock_device(udev);
	}
}


int usb_set_configuration(struct usb_device *dev, int configuration)
{
	int i, ret;
	struct usb_host_config *cp = NULL;
	struct usb_interface **new_interfaces = NULL;
	struct usb_hcd *hcd = bus_to_hcd(dev->bus);
	int n, nintf;

	if (dev->authorized == 0 || configuration == -1)
		configuration = 0;
	else {
		for (i = 0; i < dev->descriptor.bNumConfigurations; i++) {
			if (dev->config[i].desc.bConfigurationValue ==
					configuration) {
				cp = &dev->config[i];
				break;
			}
		}
	}
	if ((!cp && configuration != 0))
		return -EINVAL;

	/* The USB spec says configuration 0 means unconfigured.
	 * But if a device includes a configuration numbered 0,
	 * we will accept it as a correctly configured state.
	 * Use -1 if you really want to unconfigure the device.
	 */
	if (cp && configuration == 0)
		dev_warn(&dev->dev, "config 0 descriptor??\n");

	/* Allocate memory for new interfaces before doing anything else,
	 * so that if we run out then nothing will have changed. */
	n = nintf = 0;
	if (cp) {
		nintf = cp->desc.bNumInterfaces;
		new_interfaces = kmalloc(nintf * sizeof(*new_interfaces),
				GFP_NOIO);
		if (!new_interfaces) {
			dev_err(&dev->dev, "Out of memory\n");
			return -ENOMEM;
		}

		for (; n < nintf; ++n) {
			new_interfaces[n] = kzalloc(
					sizeof(struct usb_interface),
					GFP_NOIO);
			if (!new_interfaces[n]) {
				dev_err(&dev->dev, "Out of memory\n");
				ret = -ENOMEM;
free_interfaces:
				while (--n >= 0)
					kfree(new_interfaces[n]);
				kfree(new_interfaces);
				return ret;
			}
		}

		i = dev->bus_mA - cp->desc.bMaxPower * 2;
		if (i < 0)
			dev_warn(&dev->dev, "new config #%d exceeds power "
					"limit by %dmA\n",
					configuration, -i);
	}

	/* Wake up the device so we can send it the Set-Config request */
	ret = usb_autoresume_device(dev);
	if (ret)
		goto free_interfaces;

	/* if it's already configured, clear out old state first.
	 * getting rid of old interfaces means unbinding their drivers.
	 */
	if (dev->state != USB_STATE_ADDRESS)
		usb_disable_device(dev, 1);	/* Skip ep0 */

	/* Get rid of pending async Set-Config requests for this device */
	cancel_async_set_config(dev);

	/* Make sure we have bandwidth (and available HCD resources) for this
	 * configuration.  Remove endpoints from the schedule if we're dropping
	 * this configuration to set configuration 0.  After this point, the
	 * host controller will not allow submissions to dropped endpoints.  If
	 * this call fails, the device state is unchanged.
	 */
	mutex_lock(&hcd->bandwidth_mutex);
	ret = usb_hcd_alloc_bandwidth(dev, cp, NULL, NULL);
	if (ret < 0) {
		mutex_unlock(&hcd->bandwidth_mutex);
		usb_autosuspend_device(dev);
		goto free_interfaces;
	}

	ret = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
			      USB_REQ_SET_CONFIGURATION, 0, configuration, 0,
			      NULL, 0, USB_CTRL_SET_TIMEOUT);
	if (ret < 0) {
		/* All the old state is gone, so what else can we do?
		 * The device is probably useless now anyway.
		 */
		cp = NULL;
	}

	dev->actconfig = cp;
	if (!cp) {
		usb_set_device_state(dev, USB_STATE_ADDRESS);
		usb_hcd_alloc_bandwidth(dev, NULL, NULL, NULL);
		mutex_unlock(&hcd->bandwidth_mutex);
		usb_autosuspend_device(dev);
		goto free_interfaces;
	}
	mutex_unlock(&hcd->bandwidth_mutex);
	usb_set_device_state(dev, USB_STATE_CONFIGURED);

	/* Initialize the new interface structures and the
	 * hc/hcd/usbcore interface/endpoint state.
	 */
	for (i = 0; i < nintf; ++i) {
		struct usb_interface_cache *intfc;
		struct usb_interface *intf;
		struct usb_host_interface *alt;

		cp->interface[i] = intf = new_interfaces[i];
		intfc = cp->intf_cache[i];
		intf->altsetting = intfc->altsetting;
		intf->num_altsetting = intfc->num_altsetting;
		intf->intf_assoc = find_iad(dev, cp, i);
		kref_get(&intfc->ref);

		alt = usb_altnum_to_altsetting(intf, 0);

		/* No altsetting 0?  We'll assume the first altsetting.
		 * We could use a GetInterface call, but if a device is
		 * so non-compliant that it doesn't have altsetting 0
		 * then I wouldn't trust its reply anyway.
		 */
		if (!alt)
			alt = &intf->altsetting[0];

		intf->cur_altsetting = alt;
		usb_enable_interface(dev, intf, true);
		intf->dev.parent = &dev->dev;
		intf->dev.driver = NULL;
		intf->dev.bus = &usb_bus_type;
		intf->dev.type = &usb_if_device_type;
		intf->dev.groups = usb_interface_groups;
		intf->dev.dma_mask = dev->dev.dma_mask;
		INIT_WORK(&intf->reset_ws, __usb_queue_reset_device);
		device_initialize(&intf->dev);
		dev_set_name(&intf->dev, "%d-%s:%d.%d",
			dev->bus->busnum, dev->devpath,
			configuration, alt->desc.bInterfaceNumber);
	}
	kfree(new_interfaces);

	if (cp->string == NULL &&
			!(dev->quirks & USB_QUIRK_CONFIG_INTF_STRINGS))
		cp->string = usb_cache_string(dev, cp->desc.iConfiguration);

	/* Now that all the interfaces are set up, register them
	 * to trigger binding of drivers to interfaces.  probe()
	 * routines may install different altsettings and may
	 * claim() any interfaces not yet bound.  Many class drivers
	 * need that: CDC, audio, video, etc.
	 */
	for (i = 0; i < nintf; ++i) {
		struct usb_interface *intf = cp->interface[i];

		dev_dbg(&dev->dev,
			"adding %s (config #%d, interface %d)\n",
			dev_name(&intf->dev), configuration,
			intf->cur_altsetting->desc.bInterfaceNumber);
		device_enable_async_suspend(&intf->dev);
		ret = device_add(&intf->dev);
		if (ret != 0) {
			dev_err(&dev->dev, "device_add(%s) --> %d\n",
				dev_name(&intf->dev), ret);
			continue;
		}
		create_intf_ep_devs(intf);
	}

	usb_autosuspend_device(dev);
	return 0;
}

static LIST_HEAD(set_config_list);
static DEFINE_SPINLOCK(set_config_lock);

struct set_config_request {
	struct usb_device	*udev;
	int			config;
	struct work_struct	work;
	struct list_head	node;
};

/* Worker routine for usb_driver_set_configuration() */
static void driver_set_config_work(struct work_struct *work)
{
	struct set_config_request *req =
		container_of(work, struct set_config_request, work);
	struct usb_device *udev = req->udev;

	usb_lock_device(udev);
	spin_lock(&set_config_lock);
	list_del(&req->node);
	spin_unlock(&set_config_lock);

	if (req->config >= -1)		/* Is req still valid? */
		usb_set_configuration(udev, req->config);
	usb_unlock_device(udev);
	usb_put_dev(udev);
	kfree(req);
}

static void cancel_async_set_config(struct usb_device *udev)
{
	struct set_config_request *req;

	spin_lock(&set_config_lock);
	list_for_each_entry(req, &set_config_list, node) {
		if (req->udev == udev)
			req->config = -999;	/* Mark as cancelled */
	}
	spin_unlock(&set_config_lock);
}

int usb_driver_set_configuration(struct usb_device *udev, int config)
{
	struct set_config_request *req;

	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;
	req->udev = udev;
	req->config = config;
	INIT_WORK(&req->work, driver_set_config_work);

	spin_lock(&set_config_lock);
	list_add(&req->node, &set_config_list);
	spin_unlock(&set_config_lock);

	usb_get_dev(udev);
	schedule_work(&req->work);
	return 0;
}
EXPORT_SYMBOL_GPL(usb_driver_set_configuration);
