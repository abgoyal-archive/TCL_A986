

#include <linux/in.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/slab.h>

#include <net/datalink.h>
#include <net/p8022.h>

static int p8023_request(struct datalink_proto *dl,
			 struct sk_buff *skb, unsigned char *dest_node)
{
	struct net_device *dev = skb->dev;

	dev_hard_header(skb, dev, ETH_P_802_3, dest_node, NULL, skb->len);
	return dev_queue_xmit(skb);
}

struct datalink_proto *make_8023_client(void)
{
	struct datalink_proto *proto = kmalloc(sizeof(*proto), GFP_ATOMIC);

	if (proto) {
		proto->header_length = 0;
		proto->request	     = p8023_request;
	}
	return proto;
}

void destroy_8023_client(struct datalink_proto *dl)
{
	kfree(dl);
}

EXPORT_SYMBOL(destroy_8023_client);
EXPORT_SYMBOL(make_8023_client);

MODULE_LICENSE("GPL");
