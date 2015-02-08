

#include <linux/etherdevice.h>
#include <linux/slab.h>
#include <linux/wlp.h>

#include "wlp-internal.h"

static
void wlp_direct_assoc_frame(struct wlp *wlp, struct sk_buff *skb,
			   struct uwb_dev_addr *src)
{
	struct device *dev = &wlp->rc->uwb_dev.dev;
	struct wlp_frame_assoc *assoc = (void *) skb->data;
	struct wlp_assoc_frame_ctx *frame_ctx;

	frame_ctx = kmalloc(sizeof(*frame_ctx), GFP_ATOMIC);
	if (frame_ctx == NULL) {
		dev_err(dev, "WLP: Unable to allocate memory for association "
			"frame handling.\n");
		kfree_skb(skb);
		return;
	}
	frame_ctx->wlp = wlp;
	frame_ctx->skb = skb;
	frame_ctx->src = *src;
	switch (assoc->type) {
	case WLP_ASSOC_D1:
		INIT_WORK(&frame_ctx->ws, wlp_handle_d1_frame);
		schedule_work(&frame_ctx->ws);
		break;
	case WLP_ASSOC_E1:
		kfree_skb(skb); /* Temporary until we handle it */
		kfree(frame_ctx); /* Temporary until we handle it */
		break;
	case WLP_ASSOC_C1:
		INIT_WORK(&frame_ctx->ws, wlp_handle_c1_frame);
		schedule_work(&frame_ctx->ws);
		break;
	case WLP_ASSOC_C3:
		INIT_WORK(&frame_ctx->ws, wlp_handle_c3_frame);
		schedule_work(&frame_ctx->ws);
		break;
	default:
		dev_err(dev, "Received unexpected association frame. "
			"Type = %d \n", assoc->type);
		kfree_skb(skb);
		kfree(frame_ctx);
		break;
	}
}

static
void wlp_receive_assoc_frame(struct wlp *wlp, struct sk_buff *skb,
			     struct uwb_dev_addr *src)
{
	struct device *dev = &wlp->rc->uwb_dev.dev;
	struct wlp_frame_assoc *assoc = (void *) skb->data;
	struct wlp_session *session = wlp->session;
	u8 version;

	if (wlp_get_version(wlp, &assoc->version, &version,
			    sizeof(assoc->version)) < 0)
		goto error;
	if (version != WLP_VERSION) {
		dev_err(dev, "Unsupported WLP version in association "
			"message.\n");
		goto error;
	}
	if (session != NULL) {
		/* Function that created this session is still holding the
		 * &wlp->mutex to protect this session. */
		if (assoc->type == session->exp_message ||
		    assoc->type == WLP_ASSOC_F0) {
			if (!memcmp(&session->neighbor_addr, src,
				   sizeof(*src))) {
				session->data = skb;
				(session->cb)(wlp);
			} else {
				dev_err(dev, "Received expected message from "
					"unexpected source.  Expected message "
					"%d or F0 from %02x:%02x, but received "
					"it from %02x:%02x. Dropping.\n",
					session->exp_message,
					session->neighbor_addr.data[1],
					session->neighbor_addr.data[0],
					src->data[1], src->data[0]);
				goto error;
			}
		} else {
			dev_err(dev, "Association already in progress. "
				"Dropping.\n");
			goto error;
		}
	} else {
		wlp_direct_assoc_frame(wlp, skb, src);
	}
	return;
error:
	kfree_skb(skb);
}

static
int wlp_verify_prep_rx_frame(struct wlp *wlp, struct sk_buff *skb,
			     struct uwb_dev_addr *src)
{
	struct device *dev = &wlp->rc->uwb_dev.dev;
	int result = -EINVAL;
	struct wlp_eda_node eda_entry;
	struct wlp_frame_std_abbrv_hdr *hdr = (void *) skb->data;

	/*verify*/
	result = wlp_copy_eda_node(&wlp->eda, src, &eda_entry);
	if (result < 0) {
		if (printk_ratelimit())
			dev_err(dev, "WLP: Incoming frame is from unknown "
				"neighbor %02x:%02x.\n", src->data[1],
				src->data[0]);
		goto out;
	}
	if (hdr->tag != eda_entry.tag) {
		if (printk_ratelimit())
			dev_err(dev, "WLP: Tag of incoming frame from "
				"%02x:%02x does not match expected tag. "
				"Received 0x%02x, expected 0x%02x. \n",
				src->data[1], src->data[0], hdr->tag,
				eda_entry.tag);
		result = -EINVAL;
		goto out;
	}
	if (eda_entry.state != WLP_WSS_CONNECTED) {
		if (printk_ratelimit())
			dev_err(dev, "WLP: Incoming frame from "
				"%02x:%02x does is not from connected WSS.\n",
				src->data[1], src->data[0]);
		result = -EINVAL;
		goto out;
	}
	/*prep*/
	skb_pull(skb, sizeof(*hdr));
out:
	return result;
}

int wlp_receive_frame(struct device *dev, struct wlp *wlp, struct sk_buff *skb,
		      struct uwb_dev_addr *src)
{
	unsigned len = skb->len;
	void *ptr = skb->data;
	struct wlp_frame_hdr *hdr;
	int result = 0;

	if (len < sizeof(*hdr)) {
		dev_err(dev, "Not enough data to parse WLP header.\n");
		result = -EINVAL;
		goto out;
	}
	hdr = ptr;
	if (le16_to_cpu(hdr->mux_hdr) != WLP_PROTOCOL_ID) {
		dev_err(dev, "Not a WLP frame type.\n");
		result = -EINVAL;
		goto out;
	}
	switch (hdr->type) {
	case WLP_FRAME_STANDARD:
		if (len < sizeof(struct wlp_frame_std_abbrv_hdr)) {
			dev_err(dev, "Not enough data to parse Standard "
				"WLP header.\n");
			goto out;
		}
		result = wlp_verify_prep_rx_frame(wlp, skb, src);
		if (result < 0) {
			if (printk_ratelimit())
				dev_err(dev, "WLP: Verification of frame "
					"from neighbor %02x:%02x failed.\n",
					src->data[1], src->data[0]);
			goto out;
		}
		result = 1;
		break;
	case WLP_FRAME_ABBREVIATED:
		dev_err(dev, "Abbreviated frame received. FIXME?\n");
		kfree_skb(skb);
		break;
	case WLP_FRAME_CONTROL:
		dev_err(dev, "Control frame received. FIXME?\n");
		kfree_skb(skb);
		break;
	case WLP_FRAME_ASSOCIATION:
		if (len < sizeof(struct wlp_frame_assoc)) {
			dev_err(dev, "Not enough data to parse Association "
				"WLP header.\n");
			goto out;
		}
		wlp_receive_assoc_frame(wlp, skb, src);
		break;
	default:
		dev_err(dev, "Invalid frame received.\n");
		result = -EINVAL;
		break;
	}
out:
	if (result < 0) {
		kfree_skb(skb);
		result = 0;
	}
	return result;
}
EXPORT_SYMBOL_GPL(wlp_receive_frame);


int wlp_prepare_tx_frame(struct device *dev, struct wlp *wlp,
			 struct sk_buff *skb, struct uwb_dev_addr *dst)
{
	int result = -EINVAL;
	struct ethhdr *eth_hdr = (void *) skb->data;

	if (is_multicast_ether_addr(eth_hdr->h_dest)) {
		result = wlp_eda_for_each(&wlp->eda, wlp_wss_send_copy, skb);
		if (result < 0) {
			if (printk_ratelimit())
				dev_err(dev, "Unable to handle broadcast "
					"frame from WLP client.\n");
			goto out;
		}
		dev_kfree_skb_irq(skb);
		result = 1;
		/* Frame will be transmitted by WLP. */
	} else {
		result = wlp_eda_for_virtual(&wlp->eda, eth_hdr->h_dest, dst,
					     wlp_wss_prep_hdr, skb);
		if (unlikely(result < 0)) {
			if (printk_ratelimit())
				dev_err(dev, "Unable to prepare "
					"skb for transmission. \n");
			goto out;
		}
	}
out:
	return result;
}
EXPORT_SYMBOL_GPL(wlp_prepare_tx_frame);
