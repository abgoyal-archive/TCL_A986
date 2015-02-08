

#ifndef _DEBUG_H_
#define _DEBUG_H_

#include <linux/kernel.h>

#define USB_STORAGE "usb-storage: "

#ifdef CONFIG_USB_STORAGE_DEBUG
void usb_stor_show_command(struct scsi_cmnd *srb);
void usb_stor_show_sense( unsigned char key,
		unsigned char asc, unsigned char ascq );
#define US_DEBUGP(x...) printk( KERN_DEBUG USB_STORAGE x )
#define US_DEBUGPX(x...) printk( x )
#define US_DEBUG(x) x 
#else
#define US_DEBUGP(x...)
#define US_DEBUGPX(x...)
#define US_DEBUG(x)
#endif

#endif
