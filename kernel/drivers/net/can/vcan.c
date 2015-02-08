

#include <linux/module.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/slab.h>
#include <net/rtnetlink.h>

static __initdata const char banner[] =
	KERN_INFO "vcan: Virtual CAN interface driver\n";

MODULE_DESCRIPTION("virtual CAN interface");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Urs Thuermann <urs.thuermann@volkswagen.de>");



static int echo; /* echo testing. Default: 0 (Off) */
module_param(echo, bool, S_IRUGO);
MODULE_PARM_DESC(echo, "Echo sent frames (for testing). Default: 0 (Off)");


static void vcan_rx(struct sk_buff *skb, struct net_device *dev)
{
	struct can_frame *cf = (struct can_frame *)skb->data;
	struct net_device_stats *stats = &dev->stats;

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;

	skb->protocol  = htons(ETH_P_CAN);
	skb->pkt_type  = PACKET_BROADCAST;
	skb->dev       = dev;
	skb->ip_summed = CHECKSUM_UNNECESSARY;

	netif_rx_ni(skb);
}

static netdev_tx_t vcan_tx(struct sk_buff *skb, struct net_device *dev)
{
	struct can_frame *cf = (struct can_frame *)skb->data;
	struct net_device_stats *stats = &dev->stats;
	int loop;

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	stats->tx_packets++;
	stats->tx_bytes += cf->can_dlc;

	/* set flag whether this packet has to be looped back */
	loop = skb->pkt_type == PACKET_LOOPBACK;

	if (!echo) {
		/* no echo handling available inside this driver */

		if (loop) {
			/*
			 * only count the packets here, because the
			 * CAN core already did the echo for us
			 */
			stats->rx_packets++;
			stats->rx_bytes += cf->can_dlc;
		}
		kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	/* perform standard echo handling for CAN network interfaces */

	if (loop) {
		struct sock *srcsk = skb->sk;

		skb = skb_share_check(skb, GFP_ATOMIC);
		if (!skb)
			return NETDEV_TX_OK;

		/* receive with packet counting */
		skb->sk = srcsk;
		vcan_rx(skb, dev);
	} else {
		/* no looped packets => no counting */
		kfree_skb(skb);
	}
	return NETDEV_TX_OK;
}

static const struct net_device_ops vcan_netdev_ops = {
	.ndo_start_xmit = vcan_tx,
};

static void vcan_setup(struct net_device *dev)
{
	dev->type		= ARPHRD_CAN;
	dev->mtu		= sizeof(struct can_frame);
	dev->hard_header_len	= 0;
	dev->addr_len		= 0;
	dev->tx_queue_len	= 0;
	dev->flags		= IFF_NOARP;

	/* set flags according to driver capabilities */
	if (echo)
		dev->flags |= IFF_ECHO;

	dev->netdev_ops		= &vcan_netdev_ops;
	dev->destructor		= free_netdev;
}

static struct rtnl_link_ops vcan_link_ops __read_mostly = {
	.kind	= "vcan",
	.setup	= vcan_setup,
};

static __init int vcan_init_module(void)
{
	printk(banner);

	if (echo)
		printk(KERN_INFO "vcan: enabled echo on driver level.\n");

	return rtnl_link_register(&vcan_link_ops);
}

static __exit void vcan_cleanup_module(void)
{
	rtnl_link_unregister(&vcan_link_ops);
}

module_init(vcan_init_module);
module_exit(vcan_cleanup_module);
