

#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/string.h>
#include <linux/device.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>


int
usb_descriptor_fillbuf(void *buf, unsigned buflen,
		const struct usb_descriptor_header **src)
{
	u8	*dest = buf;

	if (!src)
		return -EINVAL;

	/* fill buffer from src[] until null descriptor ptr */
	for (; NULL != *src; src++) {
		unsigned		len = (*src)->bLength;

		if (len > buflen)
			return -EINVAL;
		memcpy(dest, *src, len);
		buflen -= len;
		dest += len;
	}
	return dest - (u8 *)buf;
}


int usb_gadget_config_buf(
	const struct usb_config_descriptor	*config,
	void					*buf,
	unsigned				length,
	const struct usb_descriptor_header	**desc
)
{
	struct usb_config_descriptor		*cp = buf;
	int					len;

	/* config descriptor first */
	if (length < USB_DT_CONFIG_SIZE || !desc)
		return -EINVAL;
	*cp = *config;

	/* then interface/endpoint/class/vendor/... */
	len = usb_descriptor_fillbuf(USB_DT_CONFIG_SIZE + (u8*)buf,
			length - USB_DT_CONFIG_SIZE, desc);
	if (len < 0)
		return len;
	len += USB_DT_CONFIG_SIZE;
	if (len > 0xffff)
		return -EINVAL;

	/* patch up the config descriptor */
	cp->bLength = USB_DT_CONFIG_SIZE;
	cp->bDescriptorType = USB_DT_CONFIG;
	cp->wTotalLength = cpu_to_le16(len);
	cp->bmAttributes |= USB_CONFIG_ATT_ONE;
	return len;
}

struct usb_descriptor_header **
usb_copy_descriptors(struct usb_descriptor_header **src)
{
	struct usb_descriptor_header **tmp;
	unsigned bytes;
	unsigned n_desc;
	void *mem;
	struct usb_descriptor_header **ret;

	/* count descriptors and their sizes; then add vector size */
	for (bytes = 0, n_desc = 0, tmp = src; *tmp; tmp++, n_desc++)
		bytes += (*tmp)->bLength;
	bytes += (n_desc + 1) * sizeof(*tmp);

	mem = kmalloc(bytes, GFP_KERNEL);
	if (!mem)
		return NULL;

	/* fill in pointers starting at "tmp",
	 * to descriptors copied starting at "mem";
	 * and return "ret"
	 */
	tmp = mem;
	ret = mem;
	mem += (n_desc + 1) * sizeof(*tmp);
	while (*src) {
		memcpy(mem, *src, (*src)->bLength);
		*tmp = mem;
		tmp++;
		mem += (*src)->bLength;
		src++;
	}
	*tmp = NULL;

	return ret;
}

struct usb_endpoint_descriptor *
usb_find_endpoint(
	struct usb_descriptor_header **src,
	struct usb_descriptor_header **copy,
	struct usb_endpoint_descriptor *match
)
{
	while (*src) {
		if (*src == (void *) match)
			return (void *)*copy;
		src++;
		copy++;
	}
	return NULL;
}
