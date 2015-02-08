

#include <asm/unaligned.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/tty_flip.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

static int debug;

/* Vendor and Product ID */
#define AIRCABLE_VID		0x16CA
#define AIRCABLE_USB_PID	0x1502

/* Protocol Stuff */
#define HCI_HEADER_LENGTH	0x4
#define TX_HEADER_0		0x20
#define TX_HEADER_1		0x29
#define RX_HEADER_0		0x00
#define RX_HEADER_1		0x20
#define HCI_COMPLETE_FRAME	64

/* rx_flags */
#define THROTTLED		0x01
#define ACTUALLY_THROTTLED	0x02

#define DRIVER_VERSION "v2.0"
#define DRIVER_AUTHOR "Naranjo, Manuel Francisco <naranjo.manuel@gmail.com>, Johan Hovold <jhovold@gmail.com>"
#define DRIVER_DESC "AIRcable USB Driver"

/* ID table that will be registered with USB core */
static const struct usb_device_id id_table[] = {
	{ USB_DEVICE(AIRCABLE_VID, AIRCABLE_USB_PID) },
	{ },
};
MODULE_DEVICE_TABLE(usb, id_table);

static int aircable_prepare_write_buffer(struct usb_serial_port *port,
						void *dest, size_t size)
{
	int count;
	unsigned char *buf = dest;

	count = kfifo_out_locked(&port->write_fifo, buf + HCI_HEADER_LENGTH,
					size - HCI_HEADER_LENGTH, &port->lock);
	buf[0] = TX_HEADER_0;
	buf[1] = TX_HEADER_1;
	put_unaligned_le16(count, &buf[2]);

	return count + HCI_HEADER_LENGTH;
}

static int aircable_probe(struct usb_serial *serial,
			  const struct usb_device_id *id)
{
	struct usb_host_interface *iface_desc = serial->interface->
								cur_altsetting;
	struct usb_endpoint_descriptor *endpoint;
	int num_bulk_out = 0;
	int i;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i].desc;
		if (usb_endpoint_is_bulk_out(endpoint)) {
			dbg("found bulk out on endpoint %d", i);
			++num_bulk_out;
		}
	}

	if (num_bulk_out == 0) {
		dbg("Invalid interface, discarding");
		return -ENODEV;
	}

	return 0;
}

static int aircable_process_packet(struct tty_struct *tty,
			struct usb_serial_port *port, int has_headers,
			char *packet, int len)
{
	if (has_headers) {
		len -= HCI_HEADER_LENGTH;
		packet += HCI_HEADER_LENGTH;
	}
	if (len <= 0) {
		dbg("%s - malformed packet", __func__);
		return 0;
	}

	tty_insert_flip_string(tty, packet, len);

	return len;
}

static void aircable_process_read_urb(struct urb *urb)
{
	struct usb_serial_port *port = urb->context;
	char *data = (char *)urb->transfer_buffer;
	struct tty_struct *tty;
	int has_headers;
	int count;
	int len;
	int i;

	tty = tty_port_tty_get(&port->port);
	if (!tty)
		return;

	has_headers = (urb->actual_length > 2 && data[0] == RX_HEADER_0);

	count = 0;
	for (i = 0; i < urb->actual_length; i += HCI_COMPLETE_FRAME) {
		len = min_t(int, urb->actual_length - i, HCI_COMPLETE_FRAME);
		count += aircable_process_packet(tty, port, has_headers,
								&data[i], len);
	}

	if (count)
		tty_flip_buffer_push(tty);
	tty_kref_put(tty);
}

static struct usb_driver aircable_driver = {
	.name =		"aircable",
	.probe =	usb_serial_probe,
	.disconnect =	usb_serial_disconnect,
	.id_table =	id_table,
	.no_dynamic_id =	1,
};

static struct usb_serial_driver aircable_device = {
	.driver = {
		.owner =	THIS_MODULE,
		.name =		"aircable",
	},
	.usb_driver = 		&aircable_driver,
	.id_table = 		id_table,
	.num_ports =		1,
	.bulk_out_size =	HCI_COMPLETE_FRAME,
	.probe =		aircable_probe,
	.process_read_urb =	aircable_process_read_urb,
	.prepare_write_buffer =	aircable_prepare_write_buffer,
	.throttle =		usb_serial_generic_throttle,
	.unthrottle =		usb_serial_generic_unthrottle,
};

static int __init aircable_init(void)
{
	int retval;
	retval = usb_serial_register(&aircable_device);
	if (retval)
		goto failed_serial_register;
	retval = usb_register(&aircable_driver);
	if (retval)
		goto failed_usb_register;
	return 0;

failed_usb_register:
	usb_serial_deregister(&aircable_device);
failed_serial_register:
	return retval;
}

static void __exit aircable_exit(void)
{
	usb_deregister(&aircable_driver);
	usb_serial_deregister(&aircable_device);
}

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

module_init(aircable_init);
module_exit(aircable_exit);

module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug enabled or not");
