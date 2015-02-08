

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/net.h>

struct stub_device {
	struct usb_interface *interface;
	struct list_head list;

	struct usbip_device ud;
	__u32 devid;

	/*
	 * stub_priv preserves private data of each urb.
	 * It is allocated as stub_priv_cache and assigned to urb->context.
	 *
	 * stub_priv is always linked to any one of 3 lists;
	 *	priv_init: linked to this until the comletion of a urb.
	 *	priv_tx  : linked to this after the completion of a urb.
	 *	priv_free: linked to this after the sending of the result.
	 *
	 * Any of these list operations should be locked by priv_lock.
	 */
	spinlock_t priv_lock;
	struct list_head priv_init;
	struct list_head priv_tx;
	struct list_head priv_free;

	/* see comments for unlinking in stub_rx.c */
	struct list_head unlink_tx;
	struct list_head unlink_free;


	wait_queue_head_t tx_waitq;
};

/* private data into urb->priv */
struct stub_priv {
	unsigned long seqnum;
	struct list_head list;
	struct stub_device *sdev;
	struct urb *urb;

	int unlinking;
};

struct stub_unlink {
	unsigned long seqnum;
	struct list_head list;
	__u32 status;
};


extern struct kmem_cache *stub_priv_cache;


/*-------------------------------------------------------------------------*/
/* prototype declarations */

/* stub_tx.c */
void stub_complete(struct urb *);
void stub_tx_loop(struct usbip_task *);

/* stub_dev.c */
extern struct usb_driver stub_driver;

/* stub_rx.c */
void stub_rx_loop(struct usbip_task *);
void stub_enqueue_ret_unlink(struct stub_device *, __u32, __u32);

/* stub_main.c */
int match_busid(const char *busid);
void stub_device_cleanup_urbs(struct stub_device *sdev);
