



#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/random.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/mutex.h>

#include <linux/usb.h>
#include <linux/fs.h>

#define DRIVER_VERSION "v0.7.5 (30/10/2000)"
#define DRIVER_AUTHOR "Henning Zabel <henning@uni-paderborn.de>"
#define DRIVER_DESC "USB Driver for Mustek MDC800 Digital Camera"

/* Vendor and Product Information */
#define MDC800_VENDOR_ID 	0x055f
#define MDC800_PRODUCT_ID	0xa800

/* Timeouts (msec) */
#define TO_DOWNLOAD_GET_READY		1500
#define TO_DOWNLOAD_GET_BUSY		1500
#define TO_WRITE_GET_READY		1000
#define TO_DEFAULT_COMMAND		5000
#define TO_READ_FROM_IRQ 		TO_DEFAULT_COMMAND
#define TO_GET_READY			TO_DEFAULT_COMMAND

/* Minor Number of the device (create with mknod /dev/mustek c 180 32) */
#define MDC800_DEVICE_MINOR_BASE 32




typedef enum {
	NOT_CONNECTED, READY, WORKING, DOWNLOAD
} mdc800_state;


/* Data for the driver */
struct mdc800_data
{
	struct usb_device *	dev;			// Device Data
	mdc800_state 		state;

	unsigned int		endpoint [4];

	struct urb *		irq_urb;
	wait_queue_head_t	irq_wait;
	int			irq_woken;
	char*			irq_urb_buffer;

	int			camera_busy;          // is camera busy ?
	int 			camera_request_ready; // Status to synchronize with irq
	char 			camera_response [8];  // last Bytes send after busy

	struct urb *   		write_urb;
	char*			write_urb_buffer;
	wait_queue_head_t	write_wait;
	int			written;


	struct urb *   		download_urb;
	char*			download_urb_buffer;
	wait_queue_head_t	download_wait;
	int			downloaded;
	int			download_left;		// Bytes left to download ?


	/* Device Data */
	char			out [64];	// Answer Buffer
	int 			out_ptr;	// Index to the first not readen byte
	int			out_count;	// Bytes in the buffer

	int			open;		// Camera device open ?
	struct mutex		io_lock;	// IO -lock

	char 			in [8];		// Command Input Buffer
	int  			in_count;

	int			pic_index;	// Cache for the Imagesize (-1 for nothing cached )
	int			pic_len;
	int			minor;
};


/* Specification of the Endpoints */
static struct usb_endpoint_descriptor mdc800_ed [4] =
{
	{ 
		.bLength = 		0,
		.bDescriptorType =	0,
		.bEndpointAddress =	0x01,
		.bmAttributes = 	0x02,
		.wMaxPacketSize =	cpu_to_le16(8),
		.bInterval = 		0,
		.bRefresh = 		0,
		.bSynchAddress = 	0,
	},
	{
		.bLength = 		0,
		.bDescriptorType = 	0,
		.bEndpointAddress = 	0x82,
		.bmAttributes = 	0x03,
		.wMaxPacketSize = 	cpu_to_le16(8),
		.bInterval = 		0,
		.bRefresh = 		0,
		.bSynchAddress = 	0,
	},
	{
		.bLength = 		0,
		.bDescriptorType = 	0,
		.bEndpointAddress = 	0x03,
		.bmAttributes = 	0x02,
		.wMaxPacketSize = 	cpu_to_le16(64),
		.bInterval = 		0,
		.bRefresh = 		0,
		.bSynchAddress = 	0,
	},
	{
		.bLength = 		0,
		.bDescriptorType = 	0,
		.bEndpointAddress = 	0x84,
		.bmAttributes = 	0x02,
		.wMaxPacketSize = 	cpu_to_le16(64),
		.bInterval = 		0,
		.bRefresh = 		0,
		.bSynchAddress = 	0,
	},
};

/* The Variable used by the driver */
static struct mdc800_data* mdc800;



static int mdc800_endpoint_equals (struct usb_endpoint_descriptor *a,struct usb_endpoint_descriptor *b)
{
	return (
		   ( a->bEndpointAddress == b->bEndpointAddress )
		&& ( a->bmAttributes     == b->bmAttributes     )
		&& ( a->wMaxPacketSize   == b->wMaxPacketSize   )
	);
}


static int mdc800_isBusy (char* ch)
{
	int i=0;
	while (i<8)
	{
		if (ch [i] != (char)0x99)
			return 0;
		i++;
	}
	return 1;
}


static int mdc800_isReady (char *ch)
{
	int i=0;
	while (i<8)
	{
		if (ch [i] != (char)0xbb)
			return 0;
		i++;
	}
	return 1;
}



static void mdc800_usb_irq (struct urb *urb)
{
	int data_received=0, wake_up;
	unsigned char* b=urb->transfer_buffer;
	struct mdc800_data* mdc800=urb->context;
	int status = urb->status;

	if (status >= 0) {

		//dbg ("%i %i %i %i %i %i %i %i \n",b[0],b[1],b[2],b[3],b[4],b[5],b[6],b[7]);

		if (mdc800_isBusy (b))
		{
			if (!mdc800->camera_busy)
			{
				mdc800->camera_busy=1;
				dbg ("gets busy");
			}
		}
		else
		{
			if (mdc800->camera_busy && mdc800_isReady (b))
			{
				mdc800->camera_busy=0;
				dbg ("gets ready");
			}
		}
		if (!(mdc800_isBusy (b) || mdc800_isReady (b)))
		{
			/* Store Data in camera_answer field */
			dbg ("%i %i %i %i %i %i %i %i ",b[0],b[1],b[2],b[3],b[4],b[5],b[6],b[7]);

			memcpy (mdc800->camera_response,b,8);
			data_received=1;
		}
	}
	wake_up= ( mdc800->camera_request_ready > 0 )
		&&
		(
			((mdc800->camera_request_ready == 1) && (!mdc800->camera_busy))
		||
			((mdc800->camera_request_ready == 2) && data_received)
		||
			((mdc800->camera_request_ready == 3) && (mdc800->camera_busy))
		||
			(status < 0)
		);

	if (wake_up)
	{
		mdc800->camera_request_ready=0;
		mdc800->irq_woken=1;
		wake_up (&mdc800->irq_wait);
	}
}


static int mdc800_usb_waitForIRQ (int mode, int msec)
{
	mdc800->camera_request_ready=1+mode;

	wait_event_timeout(mdc800->irq_wait, mdc800->irq_woken, msec*HZ/1000);
	mdc800->irq_woken = 0;

	if (mdc800->camera_request_ready>0)
	{
		mdc800->camera_request_ready=0;
		dev_err(&mdc800->dev->dev, "timeout waiting for camera.\n");
		return -1;
	}
	
	if (mdc800->state == NOT_CONNECTED)
	{
		printk(KERN_WARNING "mdc800: Camera gets disconnected "
		       "during waiting for irq.\n");
		mdc800->camera_request_ready=0;
		return -2;
	}
	
	return 0;
}


static void mdc800_usb_write_notify (struct urb *urb)
{
	struct mdc800_data* mdc800=urb->context;
	int status = urb->status;

	if (status != 0)
		dev_err(&mdc800->dev->dev,
			"writing command fails (status=%i)\n", status);
	else
		mdc800->state=READY;
	mdc800->written = 1;
	wake_up (&mdc800->write_wait);
}


static void mdc800_usb_download_notify (struct urb *urb)
{
	struct mdc800_data* mdc800=urb->context;
	int status = urb->status;

	if (status == 0) {
		/* Fill output buffer with these data */
		memcpy (mdc800->out,  urb->transfer_buffer, 64);
		mdc800->out_count=64;
		mdc800->out_ptr=0;
		mdc800->download_left-=64;
		if (mdc800->download_left == 0)
		{
			mdc800->state=READY;
		}
	} else {
		dev_err(&mdc800->dev->dev,
			"request bytes fails (status:%i)\n", status);
	}
	mdc800->downloaded = 1;
	wake_up (&mdc800->download_wait);
}



static struct usb_driver mdc800_usb_driver;
static const struct file_operations mdc800_device_ops;
static struct usb_class_driver mdc800_class = {
	.name =		"mdc800%d",
	.fops =		&mdc800_device_ops,
	.minor_base =	MDC800_DEVICE_MINOR_BASE,
};


static int mdc800_usb_probe (struct usb_interface *intf,
			       const struct usb_device_id *id)
{
	int i,j;
	struct usb_host_interface *intf_desc;
	struct usb_device *dev = interface_to_usbdev (intf);
	int irq_interval=0;
	int retval;

	dbg ("(mdc800_usb_probe) called.");


	if (mdc800->dev != NULL)
	{
		dev_warn(&intf->dev, "only one Mustek MDC800 is supported.\n");
		return -ENODEV;
	}

	if (dev->descriptor.bNumConfigurations != 1)
	{
		dev_err(&intf->dev,
			"probe fails -> wrong Number of Configuration\n");
		return -ENODEV;
	}
	intf_desc = intf->cur_altsetting;

	if (
			( intf_desc->desc.bInterfaceClass != 0xff )
		||	( intf_desc->desc.bInterfaceSubClass != 0 )
		|| ( intf_desc->desc.bInterfaceProtocol != 0 )
		|| ( intf_desc->desc.bNumEndpoints != 4)
	)
	{
		dev_err(&intf->dev, "probe fails -> wrong Interface\n");
		return -ENODEV;
	}

	/* Check the Endpoints */
	for (i=0; i<4; i++)
	{
		mdc800->endpoint[i]=-1;
		for (j=0; j<4; j++)
		{
			if (mdc800_endpoint_equals (&intf_desc->endpoint [j].desc,&mdc800_ed [i]))
			{
				mdc800->endpoint[i]=intf_desc->endpoint [j].desc.bEndpointAddress ;
				if (i==1)
				{
					irq_interval=intf_desc->endpoint [j].desc.bInterval;
				}
			}
		}
		if (mdc800->endpoint[i] == -1)
		{
			dev_err(&intf->dev, "probe fails -> Wrong Endpoints.\n");
			return -ENODEV;
		}
	}


	dev_info(&intf->dev, "Found Mustek MDC800 on USB.\n");

	mutex_lock(&mdc800->io_lock);

	retval = usb_register_dev(intf, &mdc800_class);
	if (retval) {
		dev_err(&intf->dev, "Not able to get a minor for this device.\n");
		mutex_unlock(&mdc800->io_lock);
		return -ENODEV;
	}

	mdc800->dev=dev;
	mdc800->open=0;

	/* Setup URB Structs */
	usb_fill_int_urb (
		mdc800->irq_urb,
		mdc800->dev,
		usb_rcvintpipe (mdc800->dev,mdc800->endpoint [1]),
		mdc800->irq_urb_buffer,
		8,
		mdc800_usb_irq,
		mdc800,
		irq_interval
	);

	usb_fill_bulk_urb (
		mdc800->write_urb,
		mdc800->dev,
		usb_sndbulkpipe (mdc800->dev, mdc800->endpoint[0]),
		mdc800->write_urb_buffer,
		8,
		mdc800_usb_write_notify,
		mdc800
	);

	usb_fill_bulk_urb (
		mdc800->download_urb,
		mdc800->dev,
		usb_rcvbulkpipe (mdc800->dev, mdc800->endpoint [3]),
		mdc800->download_urb_buffer,
		64,
		mdc800_usb_download_notify,
		mdc800
	);

	mdc800->state=READY;

	mutex_unlock(&mdc800->io_lock);
	
	usb_set_intfdata(intf, mdc800);
	return 0;
}


static void mdc800_usb_disconnect (struct usb_interface *intf)
{
	struct mdc800_data* mdc800 = usb_get_intfdata(intf);

	dbg ("(mdc800_usb_disconnect) called");

	if (mdc800) {
		if (mdc800->state == NOT_CONNECTED)
			return;

		usb_deregister_dev(intf, &mdc800_class);

		/* must be under lock to make sure no URB
		   is submitted after usb_kill_urb() */
		mutex_lock(&mdc800->io_lock);
		mdc800->state=NOT_CONNECTED;

		usb_kill_urb(mdc800->irq_urb);
		usb_kill_urb(mdc800->write_urb);
		usb_kill_urb(mdc800->download_urb);
		mutex_unlock(&mdc800->io_lock);

		mdc800->dev = NULL;
		usb_set_intfdata(intf, NULL);
	}
	dev_info(&intf->dev, "Mustek MDC800 disconnected from USB.\n");
}



static int mdc800_getAnswerSize (char command)
{
	switch ((unsigned char) command)
	{
		case 0x2a:
		case 0x49:
		case 0x51:
		case 0x0d:
		case 0x20:
		case 0x07:
		case 0x01:
		case 0x25:
		case 0x00:
			return 8;

		case 0x05:
		case 0x3e:
			return mdc800->pic_len;

		case 0x09:
			return 4096;

		default:
			return 0;
	}
}


static int mdc800_device_open (struct inode* inode, struct file *file)
{
	int retval=0;
	int errn=0;

	mutex_lock(&mdc800->io_lock);
	
	if (mdc800->state == NOT_CONNECTED)
	{
		errn=-EBUSY;
		goto error_out;
	}
	if (mdc800->open)
	{
		errn=-EBUSY;
		goto error_out;
	}

	mdc800->in_count=0;
	mdc800->out_count=0;
	mdc800->out_ptr=0;
	mdc800->pic_index=0;
	mdc800->pic_len=-1;
	mdc800->download_left=0;

	mdc800->camera_busy=0;
	mdc800->camera_request_ready=0;

	retval=0;
	mdc800->irq_urb->dev = mdc800->dev;
	retval = usb_submit_urb (mdc800->irq_urb, GFP_KERNEL);
	if (retval) {
		dev_err(&mdc800->dev->dev,
			"request USB irq fails (submit_retval=%i).\n", retval);
		errn = -EIO;
		goto error_out;
	}

	mdc800->open=1;
	dbg ("Mustek MDC800 device opened.");

error_out:
	mutex_unlock(&mdc800->io_lock);
	return errn;
}


static int mdc800_device_release (struct inode* inode, struct file *file)
{
	int retval=0;
	dbg ("Mustek MDC800 device closed.");

	mutex_lock(&mdc800->io_lock);
	if (mdc800->open && (mdc800->state != NOT_CONNECTED))
	{
		usb_kill_urb(mdc800->irq_urb);
		usb_kill_urb(mdc800->write_urb);
		usb_kill_urb(mdc800->download_urb);
		mdc800->open=0;
	}
	else
	{
		retval=-EIO;
	}

	mutex_unlock(&mdc800->io_lock);
	return retval;
}


static ssize_t mdc800_device_read (struct file *file, char __user *buf, size_t len, loff_t *pos)
{
	size_t left=len, sts=len; /* single transfer size */
	char __user *ptr = buf;
	int retval;

	mutex_lock(&mdc800->io_lock);
	if (mdc800->state == NOT_CONNECTED)
	{
		mutex_unlock(&mdc800->io_lock);
		return -EBUSY;
	}
	if (mdc800->state == WORKING)
	{
		printk(KERN_WARNING "mdc800: Illegal State \"working\""
		       "reached during read ?!\n");
		mutex_unlock(&mdc800->io_lock);
		return -EBUSY;
	}
	if (!mdc800->open)
	{
		mutex_unlock(&mdc800->io_lock);
		return -EBUSY;
	}

	while (left)
	{
		if (signal_pending (current)) 
		{
			mutex_unlock(&mdc800->io_lock);
			return -EINTR;
		}

		sts=left > (mdc800->out_count-mdc800->out_ptr)?mdc800->out_count-mdc800->out_ptr:left;

		if (sts <= 0)
		{
			/* Too less Data in buffer */
			if (mdc800->state == DOWNLOAD)
			{
				mdc800->out_count=0;
				mdc800->out_ptr=0;

				/* Download -> Request new bytes */
				mdc800->download_urb->dev = mdc800->dev;
				retval = usb_submit_urb (mdc800->download_urb, GFP_KERNEL);
				if (retval) {
					dev_err(&mdc800->dev->dev,
						"Can't submit download urb "
						"(retval=%i)\n", retval);
					mutex_unlock(&mdc800->io_lock);
					return len-left;
				}
				wait_event_timeout(mdc800->download_wait, mdc800->downloaded,
										TO_DOWNLOAD_GET_READY*HZ/1000);
				mdc800->downloaded = 0;
				if (mdc800->download_urb->status != 0)
				{
					dev_err(&mdc800->dev->dev,
						"request download-bytes fails "
						"(status=%i)\n",
						mdc800->download_urb->status);
					mutex_unlock(&mdc800->io_lock);
					return len-left;
				}
			}
			else
			{
				/* No more bytes -> that's an error*/
				mutex_unlock(&mdc800->io_lock);
				return -EIO;
			}
		}
		else
		{
			/* Copy Bytes */
			if (copy_to_user(ptr, &mdc800->out [mdc800->out_ptr],
						sts)) {
				mutex_unlock(&mdc800->io_lock);
				return -EFAULT;
			}
			ptr+=sts;
			left-=sts;
			mdc800->out_ptr+=sts;
		}
	}

	mutex_unlock(&mdc800->io_lock);
	return len-left;
}


static ssize_t mdc800_device_write (struct file *file, const char __user *buf, size_t len, loff_t *pos)
{
	size_t i=0;
	int retval;

	mutex_lock(&mdc800->io_lock);
	if (mdc800->state != READY)
	{
		mutex_unlock(&mdc800->io_lock);
		return -EBUSY;
	}
	if (!mdc800->open )
	{
		mutex_unlock(&mdc800->io_lock);
		return -EBUSY;
	}

	while (i<len)
	{
		unsigned char c;
		if (signal_pending (current)) 
		{
			mutex_unlock(&mdc800->io_lock);
			return -EINTR;
		}
		
		if(get_user(c, buf+i))
		{
			mutex_unlock(&mdc800->io_lock);
			return -EFAULT;
		}

		/* check for command start */
		if (c == 0x55)
		{
			mdc800->in_count=0;
			mdc800->out_count=0;
			mdc800->out_ptr=0;
			mdc800->download_left=0;
		}

		/* save command byte */
		if (mdc800->in_count < 8)
		{
			mdc800->in[mdc800->in_count] = c;
			mdc800->in_count++;
		}
		else
		{
			mutex_unlock(&mdc800->io_lock);
			return -EIO;
		}

		/* Command Buffer full ? -> send it to camera */
		if (mdc800->in_count == 8)
		{
			int answersize;

			if (mdc800_usb_waitForIRQ (0,TO_GET_READY))
			{
				dev_err(&mdc800->dev->dev,
					"Camera didn't get ready.\n");
				mutex_unlock(&mdc800->io_lock);
				return -EIO;
			}

			answersize=mdc800_getAnswerSize (mdc800->in[1]);

			mdc800->state=WORKING;
			memcpy (mdc800->write_urb->transfer_buffer, mdc800->in,8);
			mdc800->write_urb->dev = mdc800->dev;
			retval = usb_submit_urb (mdc800->write_urb, GFP_KERNEL);
			if (retval) {
				dev_err(&mdc800->dev->dev,
					"submitting write urb fails "
					"(retval=%i)\n", retval);
				mutex_unlock(&mdc800->io_lock);
				return -EIO;
			}
			wait_event_timeout(mdc800->write_wait, mdc800->written, TO_WRITE_GET_READY*HZ/1000);
			mdc800->written = 0;
			if (mdc800->state == WORKING)
			{
				usb_kill_urb(mdc800->write_urb);
				mutex_unlock(&mdc800->io_lock);
				return -EIO;
			}

			switch ((unsigned char) mdc800->in[1])
			{
				case 0x05: /* Download Image */
				case 0x3e: /* Take shot in Fine Mode (WCam Mode) */
					if (mdc800->pic_len < 0)
					{
						dev_err(&mdc800->dev->dev,
							"call 0x07 before "
							"0x05,0x3e\n");
						mdc800->state=READY;
						mutex_unlock(&mdc800->io_lock);
						return -EIO;
					}
					mdc800->pic_len=-1;

				case 0x09: /* Download Thumbnail */
					mdc800->download_left=answersize+64;
					mdc800->state=DOWNLOAD;
					mdc800_usb_waitForIRQ (0,TO_DOWNLOAD_GET_BUSY);
					break;


				default:
					if (answersize)
					{

						if (mdc800_usb_waitForIRQ (1,TO_READ_FROM_IRQ))
						{
							dev_err(&mdc800->dev->dev, "requesting answer from irq fails\n");
							mutex_unlock(&mdc800->io_lock);
							return -EIO;
						}

						/* Write dummy data, (this is ugly but part of the USB Protocol */
						/* if you use endpoint 1 as bulk and not as irq) */
						memcpy (mdc800->out, mdc800->camera_response,8);

						/* This is the interpreted answer */
						memcpy (&mdc800->out[8], mdc800->camera_response,8);

						mdc800->out_ptr=0;
						mdc800->out_count=16;

						/* Cache the Imagesize, if command was getImageSize */
						if (mdc800->in [1] == (char) 0x07)
						{
							mdc800->pic_len=(int) 65536*(unsigned char) mdc800->camera_response[0]+256*(unsigned char) mdc800->camera_response[1]+(unsigned char) mdc800->camera_response[2];

							dbg ("cached imagesize = %i",mdc800->pic_len);
						}

					}
					else
					{
						if (mdc800_usb_waitForIRQ (0,TO_DEFAULT_COMMAND))
						{
							dev_err(&mdc800->dev->dev, "Command Timeout.\n");
							mutex_unlock(&mdc800->io_lock);
							return -EIO;
						}
					}
					mdc800->state=READY;
					break;
			}
		}
		i++;
	}
	mutex_unlock(&mdc800->io_lock);
	return i;
}



/* File Operations of this drivers */
static const struct file_operations mdc800_device_ops =
{
	.owner =	THIS_MODULE,
	.read =		mdc800_device_read,
	.write =	mdc800_device_write,
	.open =		mdc800_device_open,
	.release =	mdc800_device_release,
};



static const struct usb_device_id mdc800_table[] = {
	{ USB_DEVICE(MDC800_VENDOR_ID, MDC800_PRODUCT_ID) },
	{ }						/* Terminating entry */
};

MODULE_DEVICE_TABLE (usb, mdc800_table);
static struct usb_driver mdc800_usb_driver =
{
	.name =		"mdc800",
	.probe =	mdc800_usb_probe,
	.disconnect =	mdc800_usb_disconnect,
	.id_table =	mdc800_table
};




static int __init usb_mdc800_init (void)
{
	int retval = -ENODEV;
	/* Allocate Memory */
	mdc800=kzalloc (sizeof (struct mdc800_data), GFP_KERNEL);
	if (!mdc800)
		goto cleanup_on_fail;

	mdc800->dev = NULL;
	mdc800->state=NOT_CONNECTED;
	mutex_init (&mdc800->io_lock);

	init_waitqueue_head (&mdc800->irq_wait);
	init_waitqueue_head (&mdc800->write_wait);
	init_waitqueue_head (&mdc800->download_wait);

	mdc800->irq_woken = 0;
	mdc800->downloaded = 0;
	mdc800->written = 0;

	mdc800->irq_urb_buffer=kmalloc (8, GFP_KERNEL);
	if (!mdc800->irq_urb_buffer)
		goto cleanup_on_fail;
	mdc800->write_urb_buffer=kmalloc (8, GFP_KERNEL);
	if (!mdc800->write_urb_buffer)
		goto cleanup_on_fail;
	mdc800->download_urb_buffer=kmalloc (64, GFP_KERNEL);
	if (!mdc800->download_urb_buffer)
		goto cleanup_on_fail;

	mdc800->irq_urb=usb_alloc_urb (0, GFP_KERNEL);
	if (!mdc800->irq_urb)
		goto cleanup_on_fail;
	mdc800->download_urb=usb_alloc_urb (0, GFP_KERNEL);
	if (!mdc800->download_urb)
		goto cleanup_on_fail;
	mdc800->write_urb=usb_alloc_urb (0, GFP_KERNEL);
	if (!mdc800->write_urb)
		goto cleanup_on_fail;

	/* Register the driver */
	retval = usb_register(&mdc800_usb_driver);
	if (retval)
		goto cleanup_on_fail;

	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_VERSION ":"
	       DRIVER_DESC "\n");

	return 0;

	/* Clean driver up, when something fails */

cleanup_on_fail:

	if (mdc800 != NULL)
	{
		printk(KERN_ERR "mdc800: can't alloc memory!\n");

		kfree(mdc800->download_urb_buffer);
		kfree(mdc800->write_urb_buffer);
		kfree(mdc800->irq_urb_buffer);

		usb_free_urb(mdc800->write_urb);
		usb_free_urb(mdc800->download_urb);
		usb_free_urb(mdc800->irq_urb);

		kfree (mdc800);
	}
	mdc800 = NULL;
	return retval;
}


static void __exit usb_mdc800_cleanup (void)
{
	usb_deregister (&mdc800_usb_driver);

	usb_free_urb (mdc800->irq_urb);
	usb_free_urb (mdc800->download_urb);
	usb_free_urb (mdc800->write_urb);

	kfree (mdc800->irq_urb_buffer);
	kfree (mdc800->write_urb_buffer);
	kfree (mdc800->download_urb_buffer);

	kfree (mdc800);
	mdc800 = NULL;
}

module_init (usb_mdc800_init);
module_exit (usb_mdc800_cleanup);

MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");

