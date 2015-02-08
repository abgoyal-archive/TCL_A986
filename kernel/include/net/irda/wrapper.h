

#ifndef WRAPPER_H
#define WRAPPER_H

#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>

#include <net/irda/irda_device.h>	/* iobuff_t */

#define BOF  0xc0 /* Beginning of frame */
#define XBOF 0xff
#define EOF  0xc1 /* End of frame */
#define CE   0x7d /* Control escape */

#define STA BOF  /* Start flag */
#define STO EOF  /* End flag */

#define IRDA_TRANS 0x20    /* Asynchronous transparency modifier */       

/* States for receving a frame in async mode */
enum {
	OUTSIDE_FRAME, 
	BEGIN_FRAME, 
	LINK_ESCAPE, 
	INSIDE_FRAME
};

/* Proto definitions */
int async_wrap_skb(struct sk_buff *skb, __u8 *tx_buff, int buffsize);
void async_unwrap_char(struct net_device *dev, struct net_device_stats *stats,
		       iobuff_t *buf, __u8 byte);

#endif
