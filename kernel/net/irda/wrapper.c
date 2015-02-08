

#include <linux/skbuff.h>
#include <linux/string.h>
#include <linux/module.h>
#include <asm/byteorder.h>

#include <net/irda/irda.h>
#include <net/irda/wrapper.h>
#include <net/irda/crc.h>
#include <net/irda/irlap.h>
#include <net/irda/irlap_frame.h>
#include <net/irda/irda_device.h>

/************************** FRAME WRAPPING **************************/

static inline int stuff_byte(__u8 byte, __u8 *buf)
{
	switch (byte) {
	case BOF: /* FALLTHROUGH */
	case EOF: /* FALLTHROUGH */
	case CE:
		/* Insert transparently coded */
		buf[0] = CE;               /* Send link escape */
		buf[1] = byte^IRDA_TRANS;    /* Complement bit 5 */
		return 2;
		/* break; */
	default:
		 /* Non-special value, no transparency required */
		buf[0] = byte;
		return 1;
		/* break; */
	}
}

int async_wrap_skb(struct sk_buff *skb, __u8 *tx_buff, int buffsize)
{
	struct irda_skb_cb *cb = (struct irda_skb_cb *) skb->cb;
	int xbofs;
	int i;
	int n;
	union {
		__u16 value;
		__u8 bytes[2];
	} fcs;

	/* Initialize variables */
	fcs.value = INIT_FCS;
	n = 0;

	/*
	 *  Send  XBOF's for required min. turn time and for the negotiated
	 *  additional XBOFS
	 */

	if (cb->magic != LAP_MAGIC) {
		/*
		 * This will happen for all frames sent from user-space.
		 * Nothing to worry about, but we set the default number of
		 * BOF's
		 */
		IRDA_DEBUG(1, "%s(), wrong magic in skb!\n", __func__);
		xbofs = 10;
	} else
		xbofs = cb->xbofs + cb->xbofs_delay;

	IRDA_DEBUG(4, "%s(), xbofs=%d\n", __func__, xbofs);

	/* Check that we never use more than 115 + 48 xbofs */
	if (xbofs > 163) {
		IRDA_DEBUG(0, "%s(), too many xbofs (%d)\n", __func__,
			   xbofs);
		xbofs = 163;
	}

	memset(tx_buff + n, XBOF, xbofs);
	n += xbofs;

	/* Start of packet character BOF */
	tx_buff[n++] = BOF;

	/* Insert frame and calc CRC */
	for (i=0; i < skb->len; i++) {
		/*
		 *  Check for the possibility of tx buffer overflow. We use
		 *  bufsize-5 since the maximum number of bytes that can be
		 *  transmitted after this point is 5.
		 */
		if(n >= (buffsize-5)) {
			IRDA_ERROR("%s(), tx buffer overflow (n=%d)\n",
				   __func__, n);
			return n;
		}

		n += stuff_byte(skb->data[i], tx_buff+n);
		fcs.value = irda_fcs(fcs.value, skb->data[i]);
	}

	/* Insert CRC in little endian format (LSB first) */
	fcs.value = ~fcs.value;
#ifdef __LITTLE_ENDIAN
	n += stuff_byte(fcs.bytes[0], tx_buff+n);
	n += stuff_byte(fcs.bytes[1], tx_buff+n);
#else /* ifdef __BIG_ENDIAN */
	n += stuff_byte(fcs.bytes[1], tx_buff+n);
	n += stuff_byte(fcs.bytes[0], tx_buff+n);
#endif
	tx_buff[n++] = EOF;

	return n;
}
EXPORT_SYMBOL(async_wrap_skb);

/************************* FRAME UNWRAPPING *************************/

//#define POSTPONE_RX_CRC

static inline void
async_bump(struct net_device *dev,
	   struct net_device_stats *stats,
	   iobuff_t *rx_buff)
{
	struct sk_buff *newskb;
	struct sk_buff *dataskb;
	int		docopy;

	/* Check if we need to copy the data to a new skb or not.
	 * If the driver doesn't use ZeroCopy Rx, we have to do it.
	 * With ZeroCopy Rx, the rx_buff already point to a valid
	 * skb. But, if the frame is small, it is more efficient to
	 * copy it to save memory (copy will be fast anyway - that's
	 * called Rx-copy-break). Jean II */
	docopy = ((rx_buff->skb == NULL) ||
		  (rx_buff->len < IRDA_RX_COPY_THRESHOLD));

	/* Allocate a new skb */
	newskb = dev_alloc_skb(docopy ? rx_buff->len + 1 : rx_buff->truesize);
	if (!newskb)  {
		stats->rx_dropped++;
		/* We could deliver the current skb if doing ZeroCopy Rx,
		 * but this would stall the Rx path. Better drop the
		 * packet... Jean II */
		return;
	}

	/* Align IP header to 20 bytes (i.e. increase skb->data)
	 * Note this is only useful with IrLAN, as PPP has a variable
	 * header size (2 or 1 bytes) - Jean II */
	skb_reserve(newskb, 1);

	if(docopy) {
		/* Copy data without CRC (length already checked) */
		skb_copy_to_linear_data(newskb, rx_buff->data,
					rx_buff->len - 2);
		/* Deliver this skb */
		dataskb = newskb;
	} else {
		/* We are using ZeroCopy. Deliver old skb */
		dataskb = rx_buff->skb;
		/* And hook the new skb to the rx_buff */
		rx_buff->skb = newskb;
		rx_buff->head = newskb->data;	/* NOT newskb->head */
		//printk(KERN_DEBUG "ZeroCopy : len = %d, dataskb = %p, newskb = %p\n", rx_buff->len, dataskb, newskb);
	}

	/* Set proper length on skb (without CRC) */
	skb_put(dataskb, rx_buff->len - 2);

	/* Feed it to IrLAP layer */
	dataskb->dev = dev;
	skb_reset_mac_header(dataskb);
	dataskb->protocol = htons(ETH_P_IRDA);

	netif_rx(dataskb);

	stats->rx_packets++;
	stats->rx_bytes += rx_buff->len;

	/* Clean up rx_buff (redundant with async_unwrap_bof() ???) */
	rx_buff->data = rx_buff->head;
	rx_buff->len = 0;
}

static inline void
async_unwrap_bof(struct net_device *dev,
		 struct net_device_stats *stats,
		 iobuff_t *rx_buff, __u8 byte)
{
	switch(rx_buff->state) {
	case LINK_ESCAPE:
	case INSIDE_FRAME:
		/* Not supposed to happen, the previous frame is not
		 * finished - Jean II */
		IRDA_DEBUG(1, "%s(), Discarding incomplete frame\n",
			   __func__);
		stats->rx_errors++;
		stats->rx_missed_errors++;
		irda_device_set_media_busy(dev, TRUE);
		break;

	case OUTSIDE_FRAME:
	case BEGIN_FRAME:
	default:
		/* We may receive multiple BOF at the start of frame */
		break;
	}

	/* Now receiving frame */
	rx_buff->state = BEGIN_FRAME;
	rx_buff->in_frame = TRUE;

	/* Time to initialize receive buffer */
	rx_buff->data = rx_buff->head;
	rx_buff->len = 0;
	rx_buff->fcs = INIT_FCS;
}

static inline void
async_unwrap_eof(struct net_device *dev,
		 struct net_device_stats *stats,
		 iobuff_t *rx_buff, __u8 byte)
{
#ifdef POSTPONE_RX_CRC
	int	i;
#endif

	switch(rx_buff->state) {
	case OUTSIDE_FRAME:
		/* Probably missed the BOF */
		stats->rx_errors++;
		stats->rx_missed_errors++;
		irda_device_set_media_busy(dev, TRUE);
		break;

	case BEGIN_FRAME:
	case LINK_ESCAPE:
	case INSIDE_FRAME:
	default:
		/* Note : in the case of BEGIN_FRAME and LINK_ESCAPE,
		 * the fcs will most likely not match and generate an
		 * error, as expected - Jean II */
		rx_buff->state = OUTSIDE_FRAME;
		rx_buff->in_frame = FALSE;

#ifdef POSTPONE_RX_CRC
		/* If we haven't done the CRC as we receive bytes, we
		 * must do it now... Jean II */
		for(i = 0; i < rx_buff->len; i++)
			rx_buff->fcs = irda_fcs(rx_buff->fcs,
						rx_buff->data[i]);
#endif

		/* Test FCS and signal success if the frame is good */
		if (rx_buff->fcs == GOOD_FCS) {
			/* Deliver frame */
			async_bump(dev, stats, rx_buff);
			break;
		} else {
			/* Wrong CRC, discard frame!  */
			irda_device_set_media_busy(dev, TRUE);

			IRDA_DEBUG(1, "%s(), crc error\n", __func__);
			stats->rx_errors++;
			stats->rx_crc_errors++;
		}
		break;
	}
}

static inline void
async_unwrap_ce(struct net_device *dev,
		 struct net_device_stats *stats,
		 iobuff_t *rx_buff, __u8 byte)
{
	switch(rx_buff->state) {
	case OUTSIDE_FRAME:
		/* Activate carrier sense */
		irda_device_set_media_busy(dev, TRUE);
		break;

	case LINK_ESCAPE:
		IRDA_WARNING("%s: state not defined\n", __func__);
		break;

	case BEGIN_FRAME:
	case INSIDE_FRAME:
	default:
		/* Stuffed byte coming */
		rx_buff->state = LINK_ESCAPE;
		break;
	}
}

static inline void
async_unwrap_other(struct net_device *dev,
		   struct net_device_stats *stats,
		   iobuff_t *rx_buff, __u8 byte)
{
	switch(rx_buff->state) {
		/* This is on the critical path, case are ordered by
		 * probability (most frequent first) - Jean II */
	case INSIDE_FRAME:
		/* Must be the next byte of the frame */
		if (rx_buff->len < rx_buff->truesize)  {
			rx_buff->data[rx_buff->len++] = byte;
#ifndef POSTPONE_RX_CRC
			rx_buff->fcs = irda_fcs(rx_buff->fcs, byte);
#endif
		} else {
			IRDA_DEBUG(1, "%s(), Rx buffer overflow, aborting\n",
				   __func__);
			rx_buff->state = OUTSIDE_FRAME;
		}
		break;

	case LINK_ESCAPE:
		/*
		 *  Stuffed char, complement bit 5 of byte
		 *  following CE, IrLAP p.114
		 */
		byte ^= IRDA_TRANS;
		if (rx_buff->len < rx_buff->truesize)  {
			rx_buff->data[rx_buff->len++] = byte;
#ifndef POSTPONE_RX_CRC
			rx_buff->fcs = irda_fcs(rx_buff->fcs, byte);
#endif
			rx_buff->state = INSIDE_FRAME;
		} else {
			IRDA_DEBUG(1, "%s(), Rx buffer overflow, aborting\n",
				   __func__);
			rx_buff->state = OUTSIDE_FRAME;
		}
		break;

	case OUTSIDE_FRAME:
		/* Activate carrier sense */
		if(byte != XBOF)
			irda_device_set_media_busy(dev, TRUE);
		break;

	case BEGIN_FRAME:
	default:
		rx_buff->data[rx_buff->len++] = byte;
#ifndef POSTPONE_RX_CRC
		rx_buff->fcs = irda_fcs(rx_buff->fcs, byte);
#endif
		rx_buff->state = INSIDE_FRAME;
		break;
	}
}

void async_unwrap_char(struct net_device *dev,
		       struct net_device_stats *stats,
		       iobuff_t *rx_buff, __u8 byte)
{
	switch(byte) {
	case CE:
		async_unwrap_ce(dev, stats, rx_buff, byte);
		break;
	case BOF:
		async_unwrap_bof(dev, stats, rx_buff, byte);
		break;
	case EOF:
		async_unwrap_eof(dev, stats, rx_buff, byte);
		break;
	default:
		async_unwrap_other(dev, stats, rx_buff, byte);
		break;
	}
}
EXPORT_SYMBOL(async_unwrap_char);

