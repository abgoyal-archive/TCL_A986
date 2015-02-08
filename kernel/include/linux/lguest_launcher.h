
#ifndef _LINUX_LGUEST_LAUNCHER
#define _LINUX_LGUEST_LAUNCHER
/* Everything the "lguest" userspace program needs to know. */
#include <linux/types.h>

struct lguest_device_desc {
	/* The device type: console, network, disk etc.  Type 0 terminates. */
	__u8 type;
	/* The number of virtqueues (first in config array) */
	__u8 num_vq;
	/*
	 * The number of bytes of feature bits.  Multiply by 2: one for host
	 * features and one for Guest acknowledgements.
	 */
	__u8 feature_len;
	/* The number of bytes of the config array after virtqueues. */
	__u8 config_len;
	/* A status byte, written by the Guest. */
	__u8 status;
	__u8 config[0];
};

struct lguest_vqconfig {
	/* The number of entries in the virtio_ring */
	__u16 num;
	/* The interrupt we get when something happens. */
	__u16 irq;
	/* The page number of the virtio ring for this device. */
	__u32 pfn;
};
/*:*/

/* Write command first word is a request. */
enum lguest_req
{
	LHREQ_INITIALIZE, /* + base, pfnlimit, start */
	LHREQ_GETDMA, /* No longer used */
	LHREQ_IRQ, /* + irq */
	LHREQ_BREAK, /* No longer used */
	LHREQ_EVENTFD, /* + address, fd. */
};

#define LGUEST_VRING_ALIGN	4096
#endif /* _LINUX_LGUEST_LAUNCHER */
