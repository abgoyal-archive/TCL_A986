

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/slab.h>
#include <linux/wlp.h>
#include "wlp-internal.h"


/* FIXME: cache is not purged, only on device close */

/* FIXME: does not scale, change to dynamic array */

void wlp_eda_init(struct wlp_eda *eda)
{
	INIT_LIST_HEAD(&eda->cache);
	spin_lock_init(&eda->lock);
}

void wlp_eda_release(struct wlp_eda *eda)
{
	unsigned long flags;
	struct wlp_eda_node *itr, *next;

	spin_lock_irqsave(&eda->lock, flags);
	list_for_each_entry_safe(itr, next, &eda->cache, list_node) {
		list_del(&itr->list_node);
		kfree(itr);
	}
	spin_unlock_irqrestore(&eda->lock, flags);
}

int wlp_eda_create_node(struct wlp_eda *eda,
			const unsigned char eth_addr[ETH_ALEN],
			const struct uwb_dev_addr *dev_addr)
{
	int result = 0;
	struct wlp_eda_node *itr;
	unsigned long flags;

	BUG_ON(dev_addr == NULL || eth_addr == NULL);
	spin_lock_irqsave(&eda->lock, flags);
	list_for_each_entry(itr, &eda->cache, list_node) {
		if (!memcmp(&itr->dev_addr, dev_addr, sizeof(itr->dev_addr))) {
			printk(KERN_ERR "EDA cache already contains entry "
			       "for neighbor %02x:%02x\n",
			       dev_addr->data[1], dev_addr->data[0]);
			result = -EEXIST;
			goto out_unlock;
		}
	}
	itr = kzalloc(sizeof(*itr), GFP_ATOMIC);
	if (itr != NULL) {
		memcpy(itr->eth_addr, eth_addr, sizeof(itr->eth_addr));
		itr->dev_addr = *dev_addr;
		list_add(&itr->list_node, &eda->cache);
	} else
		result = -ENOMEM;
out_unlock:
	spin_unlock_irqrestore(&eda->lock, flags);
	return result;
}

void wlp_eda_rm_node(struct wlp_eda *eda, const struct uwb_dev_addr *dev_addr)
{
	struct wlp_eda_node *itr, *next;
	unsigned long flags;

	spin_lock_irqsave(&eda->lock, flags);
	list_for_each_entry_safe(itr, next, &eda->cache, list_node) {
		if (!memcmp(&itr->dev_addr, dev_addr, sizeof(itr->dev_addr))) {
			list_del(&itr->list_node);
			kfree(itr);
			break;
		}
	}
	spin_unlock_irqrestore(&eda->lock, flags);
}

int wlp_eda_update_node(struct wlp_eda *eda,
			const struct uwb_dev_addr *dev_addr,
			struct wlp_wss *wss,
			const unsigned char virt_addr[ETH_ALEN],
			const u8 tag, const enum wlp_wss_connect state)
{
	int result = -ENOENT;
	struct wlp_eda_node *itr;
	unsigned long flags;

	spin_lock_irqsave(&eda->lock, flags);
	list_for_each_entry(itr, &eda->cache, list_node) {
		if (!memcmp(&itr->dev_addr, dev_addr, sizeof(itr->dev_addr))) {
			/* Found it, update it */
			itr->wss = wss;
			memcpy(itr->virt_addr, virt_addr,
			       sizeof(itr->virt_addr));
			itr->tag = tag;
			itr->state = state;
			result = 0;
			goto out_unlock;
		}
	}
	/* Not found */
out_unlock:
	spin_unlock_irqrestore(&eda->lock, flags);
	return result;
}

int wlp_eda_update_node_state(struct wlp_eda *eda,
			      const struct uwb_dev_addr *dev_addr,
			      const enum wlp_wss_connect state)
{
	int result = -ENOENT;
	struct wlp_eda_node *itr;
	unsigned long flags;

	spin_lock_irqsave(&eda->lock, flags);
	list_for_each_entry(itr, &eda->cache, list_node) {
		if (!memcmp(&itr->dev_addr, dev_addr, sizeof(itr->dev_addr))) {
			/* Found it, update it */
			itr->state = state;
			result = 0;
			goto out_unlock;
		}
	}
	/* Not found */
out_unlock:
	spin_unlock_irqrestore(&eda->lock, flags);
	return result;
}

int wlp_copy_eda_node(struct wlp_eda *eda, struct uwb_dev_addr *dev_addr,
		      struct wlp_eda_node *eda_entry)
{
	int result = -ENOENT;
	struct wlp_eda_node *itr;
	unsigned long flags;

	spin_lock_irqsave(&eda->lock, flags);
	list_for_each_entry(itr, &eda->cache, list_node) {
		if (!memcmp(&itr->dev_addr, dev_addr, sizeof(itr->dev_addr))) {
			*eda_entry = *itr;
			result = 0;
			goto out_unlock;
		}
	}
	/* Not found */
out_unlock:
	spin_unlock_irqrestore(&eda->lock, flags);
	return result;
}

int wlp_eda_for_each(struct wlp_eda *eda, wlp_eda_for_each_f function,
		     void *priv)
{
	int result = 0;
	struct wlp *wlp = container_of(eda, struct wlp, eda);
	struct wlp_eda_node *entry;
	unsigned long flags;

	spin_lock_irqsave(&eda->lock, flags);
	list_for_each_entry(entry, &eda->cache, list_node) {
		result = (*function)(wlp, entry, priv);
		if (result < 0)
			break;
	}
	spin_unlock_irqrestore(&eda->lock, flags);
	return result;
}

int wlp_eda_for_virtual(struct wlp_eda *eda,
			const unsigned char virt_addr[ETH_ALEN],
			struct uwb_dev_addr *dev_addr,
			wlp_eda_for_each_f function,
			void *priv)
{
	int result = 0;
	struct wlp *wlp = container_of(eda, struct wlp, eda);
	struct wlp_eda_node *itr;
	unsigned long flags;
	int found = 0;

	spin_lock_irqsave(&eda->lock, flags);
	list_for_each_entry(itr, &eda->cache, list_node) {
		if (!memcmp(itr->virt_addr, virt_addr,
			   sizeof(itr->virt_addr))) {
			result = (*function)(wlp, itr, priv);
			*dev_addr = itr->dev_addr;
			found = 1;
			break;
		}
	}
	if (!found)
		result = -ENODEV;
	spin_unlock_irqrestore(&eda->lock, flags);
	return result;
}

static const char *__wlp_wss_connect_state[] = { "WLP_WSS_UNCONNECTED",
					  "WLP_WSS_CONNECTED",
					  "WLP_WSS_CONNECT_FAILED",
};

static const char *wlp_wss_connect_state_str(unsigned id)
{
	if (id >= ARRAY_SIZE(__wlp_wss_connect_state))
		return "unknown WSS connection state";
	return __wlp_wss_connect_state[id];
}

ssize_t wlp_eda_show(struct wlp *wlp, char *buf)
{
	ssize_t result = 0;
	struct wlp_eda_node *entry;
	unsigned long flags;
	struct wlp_eda *eda = &wlp->eda;
	spin_lock_irqsave(&eda->lock, flags);
	result = scnprintf(buf, PAGE_SIZE, "#eth_addr dev_addr wss_ptr "
			   "tag state virt_addr\n");
	list_for_each_entry(entry, &eda->cache, list_node) {
		result += scnprintf(buf + result, PAGE_SIZE - result,
				    "%pM %02x:%02x %p 0x%02x %s %pM\n",
				    entry->eth_addr,
				    entry->dev_addr.data[1],
				    entry->dev_addr.data[0], entry->wss,
				    entry->tag,
				    wlp_wss_connect_state_str(entry->state),
				    entry->virt_addr);
		if (result >= PAGE_SIZE)
			break;
	}
	spin_unlock_irqrestore(&eda->lock, flags);
	return result;
}
EXPORT_SYMBOL_GPL(wlp_eda_show);

ssize_t wlp_eda_store(struct wlp *wlp, const char *buf, size_t size)
{
	ssize_t result;
	struct wlp_eda *eda = &wlp->eda;
	u8 eth_addr[6];
	struct uwb_dev_addr dev_addr;
	u8 tag;
	unsigned state;

	result = sscanf(buf, "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx "
			"%02hhx:%02hhx %02hhx %u\n",
			&eth_addr[0], &eth_addr[1],
			&eth_addr[2], &eth_addr[3],
			&eth_addr[4], &eth_addr[5],
			&dev_addr.data[1], &dev_addr.data[0], &tag, &state);
	switch (result) {
	case 6: /* no dev addr specified -- remove entry NOT IMPLEMENTED */
		/*result = wlp_eda_rm(eda, eth_addr, &dev_addr);*/
		result = -ENOSYS;
		break;
	case 10:
		state = state >= 1 ? 1 : 0;
		result = wlp_eda_create_node(eda, eth_addr, &dev_addr);
		if (result < 0 && result != -EEXIST)
			goto error;
		/* Set virtual addr to be same as MAC */
		result = wlp_eda_update_node(eda, &dev_addr, &wlp->wss,
					     eth_addr, tag, state);
		if (result < 0)
			goto error;
		break;
	default: /* bad format */
		result = -EINVAL;
	}
error:
	return result < 0 ? result : size;
}
EXPORT_SYMBOL_GPL(wlp_eda_store);
