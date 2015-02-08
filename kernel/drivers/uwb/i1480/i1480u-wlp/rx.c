

#include <linux/gfp.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include "i1480u-wlp.h"

int i1480u_rx_setup(struct i1480u *i1480u)
{
	int result, cnt;
	struct device *dev = &i1480u->usb_iface->dev;
	struct net_device *net_dev = i1480u->net_dev;
	struct usb_endpoint_descriptor *epd;
	struct sk_buff *skb;

	/* Alloc RX stuff */
	i1480u->rx_skb = NULL;	/* not in process of receiving packet */
	result = -ENOMEM;
	epd = &i1480u->usb_iface->cur_altsetting->endpoint[1].desc;
	for (cnt = 0; cnt < i1480u_RX_BUFS; cnt++) {
		struct i1480u_rx_buf *rx_buf = &i1480u->rx_buf[cnt];
		rx_buf->i1480u = i1480u;
		skb = dev_alloc_skb(i1480u_MAX_RX_PKT_SIZE);
		if (!skb) {
			dev_err(dev,
				"RX: cannot allocate RX buffer %d\n", cnt);
			result = -ENOMEM;
			goto error;
		}
		skb->dev = net_dev;
		skb->ip_summed = CHECKSUM_NONE;
		skb_reserve(skb, 2);
		rx_buf->data = skb;
		rx_buf->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (unlikely(rx_buf->urb == NULL)) {
			dev_err(dev, "RX: cannot allocate URB %d\n", cnt);
			result = -ENOMEM;
			goto error;
		}
		usb_fill_bulk_urb(rx_buf->urb, i1480u->usb_dev,
			  usb_rcvbulkpipe(i1480u->usb_dev, epd->bEndpointAddress),
			  rx_buf->data->data, i1480u_MAX_RX_PKT_SIZE - 2,
			  i1480u_rx_cb, rx_buf);
		result = usb_submit_urb(rx_buf->urb, GFP_NOIO);
		if (unlikely(result < 0)) {
			dev_err(dev, "RX: cannot submit URB %d: %d\n",
				cnt, result);
			goto error;
		}
	}
	return 0;

error:
	i1480u_rx_release(i1480u);
	return result;
}


/* Release resources associated to the rx context */
void i1480u_rx_release(struct i1480u *i1480u)
{
	int cnt;
	for (cnt = 0; cnt < i1480u_RX_BUFS; cnt++) {
		if (i1480u->rx_buf[cnt].data)
			dev_kfree_skb(i1480u->rx_buf[cnt].data);
		if (i1480u->rx_buf[cnt].urb) {
			usb_kill_urb(i1480u->rx_buf[cnt].urb);
			usb_free_urb(i1480u->rx_buf[cnt].urb);
		}
	}
	if (i1480u->rx_skb != NULL)
		dev_kfree_skb(i1480u->rx_skb);
}

static
void i1480u_rx_unlink_urbs(struct i1480u *i1480u)
{
	int cnt;
	for (cnt = 0; cnt < i1480u_RX_BUFS; cnt++) {
		if (i1480u->rx_buf[cnt].urb)
			usb_unlink_urb(i1480u->rx_buf[cnt].urb);
	}
}

/* Fix an out-of-sequence packet */
#define i1480u_fix(i1480u, msg...)			\
do {							\
	if (printk_ratelimit())				\
		dev_err(&i1480u->usb_iface->dev, msg);	\
	dev_kfree_skb_irq(i1480u->rx_skb);		\
	i1480u->rx_skb = NULL;				\
	i1480u->rx_untd_pkt_size = 0;			\
} while (0)


/* Drop an out-of-sequence packet */
#define i1480u_drop(i1480u, msg...)			\
do {							\
	if (printk_ratelimit())				\
		dev_err(&i1480u->usb_iface->dev, msg);	\
	i1480u->net_dev->stats.rx_dropped++;			\
} while (0)




static
void i1480u_skb_deliver(struct i1480u *i1480u)
{
	int should_parse;
	struct net_device *net_dev = i1480u->net_dev;
	struct device *dev = &i1480u->usb_iface->dev;

	should_parse = wlp_receive_frame(dev, &i1480u->wlp, i1480u->rx_skb,
					 &i1480u->rx_srcaddr);
	if (!should_parse)
		goto out;
	i1480u->rx_skb->protocol = eth_type_trans(i1480u->rx_skb, net_dev);
	net_dev->stats.rx_packets++;
	net_dev->stats.rx_bytes += i1480u->rx_untd_pkt_size;

	netif_rx(i1480u->rx_skb);		/* deliver */
out:
	i1480u->rx_skb = NULL;
	i1480u->rx_untd_pkt_size = 0;
}


static
void i1480u_rx_buffer(struct i1480u_rx_buf *rx_buf)
{
	unsigned pkt_completed = 0;	/* !0 when we got all pkt fragments */
	size_t untd_hdr_size, untd_frg_size;
	size_t i1480u_hdr_size;
	struct wlp_rx_hdr *i1480u_hdr = NULL;

	struct i1480u *i1480u = rx_buf->i1480u;
	struct sk_buff *skb = rx_buf->data;
	int size_left = rx_buf->urb->actual_length;
	void *ptr = rx_buf->urb->transfer_buffer; /* also rx_buf->data->data */
	struct untd_hdr *untd_hdr;

	struct net_device *net_dev = i1480u->net_dev;
	struct device *dev = &i1480u->usb_iface->dev;
	struct sk_buff *new_skb;

#if 0
	dev_fnstart(dev,
		    "(i1480u %p ptr %p size_left %zu)\n", i1480u, ptr, size_left);
	dev_err(dev, "RX packet, %zu bytes\n", size_left);
	dump_bytes(dev, ptr, size_left);
#endif
	i1480u_hdr_size = sizeof(struct wlp_rx_hdr);

	while (size_left > 0) {
		if (pkt_completed) {
			i1480u_drop(i1480u, "RX: fragment follows completed"
					 "packet in same buffer. Dropping\n");
			break;
		}
		untd_hdr = ptr;
		if (size_left < sizeof(*untd_hdr)) {	/*  Check the UNTD header */
			i1480u_drop(i1480u, "RX: short UNTD header! Dropping\n");
			goto out;
		}
		if (unlikely(untd_hdr_rx_tx(untd_hdr) == 0)) {	/* Paranoia: TX set? */
			i1480u_drop(i1480u, "RX: TX bit set! Dropping\n");
			goto out;
		}
		switch (untd_hdr_type(untd_hdr)) {	/* Check the UNTD header type */
		case i1480u_PKT_FRAG_1ST: {
			struct untd_hdr_1st *untd_hdr_1st = (void *) untd_hdr;
			dev_dbg(dev, "1st fragment\n");
			untd_hdr_size = sizeof(struct untd_hdr_1st);
			if (i1480u->rx_skb != NULL)
				i1480u_fix(i1480u, "RX: 1st fragment out of "
					"sequence! Fixing\n");
			if (size_left < untd_hdr_size + i1480u_hdr_size) {
				i1480u_drop(i1480u, "RX: short 1st fragment! "
					"Dropping\n");
				goto out;
			}
			i1480u->rx_untd_pkt_size = le16_to_cpu(untd_hdr->len)
						 - i1480u_hdr_size;
			untd_frg_size = le16_to_cpu(untd_hdr_1st->fragment_len);
			if (size_left < untd_hdr_size + untd_frg_size) {
				i1480u_drop(i1480u,
					    "RX: short payload! Dropping\n");
				goto out;
			}
			i1480u->rx_skb = skb;
			i1480u_hdr = (void *) untd_hdr_1st + untd_hdr_size;
			i1480u->rx_srcaddr = i1480u_hdr->srcaddr;
			skb_put(i1480u->rx_skb, untd_hdr_size + untd_frg_size);
			skb_pull(i1480u->rx_skb, untd_hdr_size + i1480u_hdr_size);
			stats_add_sample(&i1480u->lqe_stats, (s8) i1480u_hdr->LQI - 7);
			stats_add_sample(&i1480u->rssi_stats, i1480u_hdr->RSSI + 18);
			rx_buf->data = NULL; /* need to create new buffer */
			break;
		}
		case i1480u_PKT_FRAG_NXT: {
			dev_dbg(dev, "nxt fragment\n");
			untd_hdr_size = sizeof(struct untd_hdr_rst);
			if (i1480u->rx_skb == NULL) {
				i1480u_drop(i1480u, "RX: next fragment out of "
					    "sequence! Dropping\n");
				goto out;
			}
			if (size_left < untd_hdr_size) {
				i1480u_drop(i1480u, "RX: short NXT fragment! "
					    "Dropping\n");
				goto out;
			}
			untd_frg_size = le16_to_cpu(untd_hdr->len);
			if (size_left < untd_hdr_size + untd_frg_size) {
				i1480u_drop(i1480u,
					    "RX: short payload! Dropping\n");
				goto out;
			}
			memmove(skb_put(i1480u->rx_skb, untd_frg_size),
					ptr + untd_hdr_size, untd_frg_size);
			break;
		}
		case i1480u_PKT_FRAG_LST: {
			dev_dbg(dev, "Lst fragment\n");
			untd_hdr_size = sizeof(struct untd_hdr_rst);
			if (i1480u->rx_skb == NULL) {
				i1480u_drop(i1480u, "RX: last fragment out of "
					    "sequence! Dropping\n");
				goto out;
			}
			if (size_left < untd_hdr_size) {
				i1480u_drop(i1480u, "RX: short LST fragment! "
					    "Dropping\n");
				goto out;
			}
			untd_frg_size = le16_to_cpu(untd_hdr->len);
			if (size_left < untd_frg_size + untd_hdr_size) {
				i1480u_drop(i1480u,
					    "RX: short payload! Dropping\n");
				goto out;
			}
			memmove(skb_put(i1480u->rx_skb, untd_frg_size),
					ptr + untd_hdr_size, untd_frg_size);
			pkt_completed = 1;
			break;
		}
		case i1480u_PKT_FRAG_CMP: {
			dev_dbg(dev, "cmp fragment\n");
			untd_hdr_size = sizeof(struct untd_hdr_cmp);
			if (i1480u->rx_skb != NULL)
				i1480u_fix(i1480u, "RX: fix out-of-sequence CMP"
					   " fragment!\n");
			if (size_left < untd_hdr_size + i1480u_hdr_size) {
				i1480u_drop(i1480u, "RX: short CMP fragment! "
					    "Dropping\n");
				goto out;
			}
			i1480u->rx_untd_pkt_size = le16_to_cpu(untd_hdr->len);
			untd_frg_size = i1480u->rx_untd_pkt_size;
			if (size_left < i1480u->rx_untd_pkt_size + untd_hdr_size) {
				i1480u_drop(i1480u,
					    "RX: short payload! Dropping\n");
				goto out;
			}
			i1480u->rx_skb = skb;
			i1480u_hdr = (void *) untd_hdr + untd_hdr_size;
			i1480u->rx_srcaddr = i1480u_hdr->srcaddr;
			stats_add_sample(&i1480u->lqe_stats, (s8) i1480u_hdr->LQI - 7);
			stats_add_sample(&i1480u->rssi_stats, i1480u_hdr->RSSI + 18);
			skb_put(i1480u->rx_skb, untd_hdr_size + i1480u->rx_untd_pkt_size);
			skb_pull(i1480u->rx_skb, untd_hdr_size + i1480u_hdr_size);
			rx_buf->data = NULL;	/* for hand off skb to network stack */
			pkt_completed = 1;
			i1480u->rx_untd_pkt_size -= i1480u_hdr_size; /* accurate stat */
			break;
		}
		default:
			i1480u_drop(i1480u, "RX: unknown packet type %u! "
				    "Dropping\n", untd_hdr_type(untd_hdr));
			goto out;
		}
		size_left -= untd_hdr_size + untd_frg_size;
		if (size_left > 0)
			ptr += untd_hdr_size + untd_frg_size;
	}
	if (pkt_completed)
		i1480u_skb_deliver(i1480u);
out:
	/* recreate needed RX buffers*/
	if (rx_buf->data == NULL) {
		/* buffer is being used to receive packet, create new */
		new_skb = dev_alloc_skb(i1480u_MAX_RX_PKT_SIZE);
		if (!new_skb) {
			if (printk_ratelimit())
				dev_err(dev,
				"RX: cannot allocate RX buffer\n");
		} else {
			new_skb->dev = net_dev;
			new_skb->ip_summed = CHECKSUM_NONE;
			skb_reserve(new_skb, 2);
			rx_buf->data = new_skb;
		}
	}
	return;
}


void i1480u_rx_cb(struct urb *urb)
{
	int result;
	int do_parse_buffer = 1;
	struct i1480u_rx_buf *rx_buf = urb->context;
	struct i1480u *i1480u = rx_buf->i1480u;
	struct device *dev = &i1480u->usb_iface->dev;
	unsigned long flags;
	u8 rx_buf_idx = rx_buf - i1480u->rx_buf;

	switch (urb->status) {
	case 0:
		break;
	case -ECONNRESET:	/* Not an error, but a controlled situation; */
	case -ENOENT:		/* (we killed the URB)...so, no broadcast */
	case -ESHUTDOWN:	/* going away! */
		dev_err(dev, "RX URB[%u]: goind down %d\n",
			rx_buf_idx, urb->status);
		goto error;
	default:
		dev_err(dev, "RX URB[%u]: unknown status %d\n",
			rx_buf_idx, urb->status);
		if (edc_inc(&i1480u->rx_errors, EDC_MAX_ERRORS,
					EDC_ERROR_TIMEFRAME)) {
			dev_err(dev, "RX: max acceptable errors exceeded,"
					" resetting device.\n");
			i1480u_rx_unlink_urbs(i1480u);
			wlp_reset_all(&i1480u->wlp);
			goto error;
		}
		do_parse_buffer = 0;
		break;
	}
	spin_lock_irqsave(&i1480u->lock, flags);
	/* chew the data fragments, extract network packets */
	if (do_parse_buffer) {
		i1480u_rx_buffer(rx_buf);
		if (rx_buf->data) {
			rx_buf->urb->transfer_buffer = rx_buf->data->data;
			result = usb_submit_urb(rx_buf->urb, GFP_ATOMIC);
			if (result < 0) {
				dev_err(dev, "RX URB[%u]: cannot submit %d\n",
					rx_buf_idx, result);
			}
		}
	}
	spin_unlock_irqrestore(&i1480u->lock, flags);
error:
	return;
}

