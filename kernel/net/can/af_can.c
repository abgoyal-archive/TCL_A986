

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/rcupdate.h>
#include <linux/uaccess.h>
#include <linux/net.h>
#include <linux/netdevice.h>
#include <linux/socket.h>
#include <linux/if_ether.h>
#include <linux/if_arp.h>
#include <linux/skbuff.h>
#include <linux/can.h>
#include <linux/can/core.h>
#include <net/net_namespace.h>
#include <net/sock.h>

#include "af_can.h"

static __initdata const char banner[] = KERN_INFO
	"can: controller area network core (" CAN_VERSION_STRING ")\n";

MODULE_DESCRIPTION("Controller Area Network PF_CAN core");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Urs Thuermann <urs.thuermann@volkswagen.de>, "
	      "Oliver Hartkopp <oliver.hartkopp@volkswagen.de>");

MODULE_ALIAS_NETPROTO(PF_CAN);

static int stats_timer __read_mostly = 1;
module_param(stats_timer, int, S_IRUGO);
MODULE_PARM_DESC(stats_timer, "enable timer for statistics (default:on)");

/* receive filters subscribed for 'all' CAN devices */
struct dev_rcv_lists can_rx_alldev_list;
static DEFINE_SPINLOCK(can_rcvlists_lock);

static struct kmem_cache *rcv_cache __read_mostly;

/* table of registered CAN protocols */
static struct can_proto *proto_tab[CAN_NPROTO] __read_mostly;
static DEFINE_SPINLOCK(proto_tab_lock);

struct timer_list can_stattimer;   /* timer for statistics update */
struct s_stats    can_stats;       /* packet statistics */
struct s_pstats   can_pstats;      /* receive list statistics */


static int can_ioctl(struct socket *sock, unsigned int cmd, unsigned long arg)
{
	struct sock *sk = sock->sk;

	switch (cmd) {

	case SIOCGSTAMP:
		return sock_get_timestamp(sk, (struct timeval __user *)arg);

	default:
		return -ENOIOCTLCMD;
	}
}

static void can_sock_destruct(struct sock *sk)
{
	skb_queue_purge(&sk->sk_receive_queue);
}

static int can_create(struct net *net, struct socket *sock, int protocol,
		      int kern)
{
	struct sock *sk;
	struct can_proto *cp;
	int err = 0;

	sock->state = SS_UNCONNECTED;

	if (protocol < 0 || protocol >= CAN_NPROTO)
		return -EINVAL;

	if (!net_eq(net, &init_net))
		return -EAFNOSUPPORT;

#ifdef CONFIG_MODULES
	/* try to load protocol module kernel is modular */
	if (!proto_tab[protocol]) {
		err = request_module("can-proto-%d", protocol);

		/*
		 * In case of error we only print a message but don't
		 * return the error code immediately.  Below we will
		 * return -EPROTONOSUPPORT
		 */
		if (err && printk_ratelimit())
			printk(KERN_ERR "can: request_module "
			       "(can-proto-%d) failed.\n", protocol);
	}
#endif

	spin_lock(&proto_tab_lock);
	cp = proto_tab[protocol];
	if (cp && !try_module_get(cp->prot->owner))
		cp = NULL;
	spin_unlock(&proto_tab_lock);

	/* check for available protocol and correct usage */

	if (!cp)
		return -EPROTONOSUPPORT;

	if (cp->type != sock->type) {
		err = -EPROTONOSUPPORT;
		goto errout;
	}

	sock->ops = cp->ops;

	sk = sk_alloc(net, PF_CAN, GFP_KERNEL, cp->prot);
	if (!sk) {
		err = -ENOMEM;
		goto errout;
	}

	sock_init_data(sock, sk);
	sk->sk_destruct = can_sock_destruct;

	if (sk->sk_prot->init)
		err = sk->sk_prot->init(sk);

	if (err) {
		/* release sk on errors */
		sock_orphan(sk);
		sock_put(sk);
	}

 errout:
	module_put(cp->prot->owner);
	return err;
}


int can_send(struct sk_buff *skb, int loop)
{
	struct sk_buff *newskb = NULL;
	struct can_frame *cf = (struct can_frame *)skb->data;
	int err;

	if (skb->len != sizeof(struct can_frame) || cf->can_dlc > 8) {
		kfree_skb(skb);
		return -EINVAL;
	}

	if (skb->dev->type != ARPHRD_CAN) {
		kfree_skb(skb);
		return -EPERM;
	}

	if (!(skb->dev->flags & IFF_UP)) {
		kfree_skb(skb);
		return -ENETDOWN;
	}

	skb->protocol = htons(ETH_P_CAN);
	skb_reset_network_header(skb);
	skb_reset_transport_header(skb);

	if (loop) {
		/* local loopback of sent CAN frames */

		/* indication for the CAN driver: do loopback */
		skb->pkt_type = PACKET_LOOPBACK;

		/*
		 * The reference to the originating sock may be required
		 * by the receiving socket to check whether the frame is
		 * its own. Example: can_raw sockopt CAN_RAW_RECV_OWN_MSGS
		 * Therefore we have to ensure that skb->sk remains the
		 * reference to the originating sock by restoring skb->sk
		 * after each skb_clone() or skb_orphan() usage.
		 */

		if (!(skb->dev->flags & IFF_ECHO)) {
			/*
			 * If the interface is not capable to do loopback
			 * itself, we do it here.
			 */
			newskb = skb_clone(skb, GFP_ATOMIC);
			if (!newskb) {
				kfree_skb(skb);
				return -ENOMEM;
			}

			newskb->sk = skb->sk;
			newskb->ip_summed = CHECKSUM_UNNECESSARY;
			newskb->pkt_type = PACKET_BROADCAST;
		}
	} else {
		/* indication for the CAN driver: no loopback required */
		skb->pkt_type = PACKET_HOST;
	}

	/* send to netdevice */
	err = dev_queue_xmit(skb);
	if (err > 0)
		err = net_xmit_errno(err);

	if (err) {
		kfree_skb(newskb);
		return err;
	}

	if (newskb)
		netif_rx_ni(newskb);

	/* update statistics */
	can_stats.tx_frames++;
	can_stats.tx_frames_delta++;

	return 0;
}
EXPORT_SYMBOL(can_send);


static struct dev_rcv_lists *find_dev_rcv_lists(struct net_device *dev)
{
	if (!dev)
		return &can_rx_alldev_list;
	else
		return (struct dev_rcv_lists *)dev->ml_priv;
}

static struct hlist_head *find_rcv_list(canid_t *can_id, canid_t *mask,
					struct dev_rcv_lists *d)
{
	canid_t inv = *can_id & CAN_INV_FILTER; /* save flag before masking */

	/* filter for error frames in extra filterlist */
	if (*mask & CAN_ERR_FLAG) {
		/* clear CAN_ERR_FLAG in filter entry */
		*mask &= CAN_ERR_MASK;
		return &d->rx[RX_ERR];
	}

	/* with cleared CAN_ERR_FLAG we have a simple mask/value filterpair */

#define CAN_EFF_RTR_FLAGS (CAN_EFF_FLAG | CAN_RTR_FLAG)

	/* ensure valid values in can_mask for 'SFF only' frame filtering */
	if ((*mask & CAN_EFF_FLAG) && !(*can_id & CAN_EFF_FLAG))
		*mask &= (CAN_SFF_MASK | CAN_EFF_RTR_FLAGS);

	/* reduce condition testing at receive time */
	*can_id &= *mask;

	/* inverse can_id/can_mask filter */
	if (inv)
		return &d->rx[RX_INV];

	/* mask == 0 => no condition testing at receive time */
	if (!(*mask))
		return &d->rx[RX_ALL];

	/* extra filterlists for the subscription of a single non-RTR can_id */
	if (((*mask & CAN_EFF_RTR_FLAGS) == CAN_EFF_RTR_FLAGS) &&
	    !(*can_id & CAN_RTR_FLAG)) {

		if (*can_id & CAN_EFF_FLAG) {
			if (*mask == (CAN_EFF_MASK | CAN_EFF_RTR_FLAGS)) {
				/* RFC: a future use-case for hash-tables? */
				return &d->rx[RX_EFF];
			}
		} else {
			if (*mask == (CAN_SFF_MASK | CAN_EFF_RTR_FLAGS))
				return &d->rx_sff[*can_id];
		}
	}

	/* default: filter via can_id/can_mask */
	return &d->rx[RX_FIL];
}

int can_rx_register(struct net_device *dev, canid_t can_id, canid_t mask,
		    void (*func)(struct sk_buff *, void *), void *data,
		    char *ident)
{
	struct receiver *r;
	struct hlist_head *rl;
	struct dev_rcv_lists *d;
	int err = 0;

	/* insert new receiver  (dev,canid,mask) -> (func,data) */

	if (dev && dev->type != ARPHRD_CAN)
		return -ENODEV;

	r = kmem_cache_alloc(rcv_cache, GFP_KERNEL);
	if (!r)
		return -ENOMEM;

	spin_lock(&can_rcvlists_lock);

	d = find_dev_rcv_lists(dev);
	if (d) {
		rl = find_rcv_list(&can_id, &mask, d);

		r->can_id  = can_id;
		r->mask    = mask;
		r->matches = 0;
		r->func    = func;
		r->data    = data;
		r->ident   = ident;

		hlist_add_head_rcu(&r->list, rl);
		d->entries++;

		can_pstats.rcv_entries++;
		if (can_pstats.rcv_entries_max < can_pstats.rcv_entries)
			can_pstats.rcv_entries_max = can_pstats.rcv_entries;
	} else {
		kmem_cache_free(rcv_cache, r);
		err = -ENODEV;
	}

	spin_unlock(&can_rcvlists_lock);

	return err;
}
EXPORT_SYMBOL(can_rx_register);

static void can_rx_delete_receiver(struct rcu_head *rp)
{
	struct receiver *r = container_of(rp, struct receiver, rcu);

	kmem_cache_free(rcv_cache, r);
}

void can_rx_unregister(struct net_device *dev, canid_t can_id, canid_t mask,
		       void (*func)(struct sk_buff *, void *), void *data)
{
	struct receiver *r = NULL;
	struct hlist_head *rl;
	struct hlist_node *next;
	struct dev_rcv_lists *d;

	if (dev && dev->type != ARPHRD_CAN)
		return;

	spin_lock(&can_rcvlists_lock);

	d = find_dev_rcv_lists(dev);
	if (!d) {
		printk(KERN_ERR "BUG: receive list not found for "
		       "dev %s, id %03X, mask %03X\n",
		       DNAME(dev), can_id, mask);
		goto out;
	}

	rl = find_rcv_list(&can_id, &mask, d);

	/*
	 * Search the receiver list for the item to delete.  This should
	 * exist, since no receiver may be unregistered that hasn't
	 * been registered before.
	 */

	hlist_for_each_entry_rcu(r, next, rl, list) {
		if (r->can_id == can_id && r->mask == mask &&
		    r->func == func && r->data == data)
			break;
	}

	/*
	 * Check for bugs in CAN protocol implementations:
	 * If no matching list item was found, the list cursor variable next
	 * will be NULL, while r will point to the last item of the list.
	 */

	if (!next) {
		printk(KERN_ERR "BUG: receive list entry not found for "
		       "dev %s, id %03X, mask %03X\n",
		       DNAME(dev), can_id, mask);
		r = NULL;
		goto out;
	}

	hlist_del_rcu(&r->list);
	d->entries--;

	if (can_pstats.rcv_entries > 0)
		can_pstats.rcv_entries--;

	/* remove device structure requested by NETDEV_UNREGISTER */
	if (d->remove_on_zero_entries && !d->entries) {
		kfree(d);
		dev->ml_priv = NULL;
	}

 out:
	spin_unlock(&can_rcvlists_lock);

	/* schedule the receiver item for deletion */
	if (r)
		call_rcu(&r->rcu, can_rx_delete_receiver);
}
EXPORT_SYMBOL(can_rx_unregister);

static inline void deliver(struct sk_buff *skb, struct receiver *r)
{
	r->func(skb, r->data);
	r->matches++;
}

static int can_rcv_filter(struct dev_rcv_lists *d, struct sk_buff *skb)
{
	struct receiver *r;
	struct hlist_node *n;
	int matches = 0;
	struct can_frame *cf = (struct can_frame *)skb->data;
	canid_t can_id = cf->can_id;

	if (d->entries == 0)
		return 0;

	if (can_id & CAN_ERR_FLAG) {
		/* check for error frame entries only */
		hlist_for_each_entry_rcu(r, n, &d->rx[RX_ERR], list) {
			if (can_id & r->mask) {
				deliver(skb, r);
				matches++;
			}
		}
		return matches;
	}

	/* check for unfiltered entries */
	hlist_for_each_entry_rcu(r, n, &d->rx[RX_ALL], list) {
		deliver(skb, r);
		matches++;
	}

	/* check for can_id/mask entries */
	hlist_for_each_entry_rcu(r, n, &d->rx[RX_FIL], list) {
		if ((can_id & r->mask) == r->can_id) {
			deliver(skb, r);
			matches++;
		}
	}

	/* check for inverted can_id/mask entries */
	hlist_for_each_entry_rcu(r, n, &d->rx[RX_INV], list) {
		if ((can_id & r->mask) != r->can_id) {
			deliver(skb, r);
			matches++;
		}
	}

	/* check filterlists for single non-RTR can_ids */
	if (can_id & CAN_RTR_FLAG)
		return matches;

	if (can_id & CAN_EFF_FLAG) {
		hlist_for_each_entry_rcu(r, n, &d->rx[RX_EFF], list) {
			if (r->can_id == can_id) {
				deliver(skb, r);
				matches++;
			}
		}
	} else {
		can_id &= CAN_SFF_MASK;
		hlist_for_each_entry_rcu(r, n, &d->rx_sff[can_id], list) {
			deliver(skb, r);
			matches++;
		}
	}

	return matches;
}

static int can_rcv(struct sk_buff *skb, struct net_device *dev,
		   struct packet_type *pt, struct net_device *orig_dev)
{
	struct dev_rcv_lists *d;
	struct can_frame *cf = (struct can_frame *)skb->data;
	int matches;

	if (!net_eq(dev_net(dev), &init_net))
		goto drop;

	if (WARN_ONCE(dev->type != ARPHRD_CAN ||
		      skb->len != sizeof(struct can_frame) ||
		      cf->can_dlc > 8,
		      "PF_CAN: dropped non conform skbuf: "
		      "dev type %d, len %d, can_dlc %d\n",
		      dev->type, skb->len, cf->can_dlc))
		goto drop;

	/* update statistics */
	can_stats.rx_frames++;
	can_stats.rx_frames_delta++;

	rcu_read_lock();

	/* deliver the packet to sockets listening on all devices */
	matches = can_rcv_filter(&can_rx_alldev_list, skb);

	/* find receive list for this device */
	d = find_dev_rcv_lists(dev);
	if (d)
		matches += can_rcv_filter(d, skb);

	rcu_read_unlock();

	/* consume the skbuff allocated by the netdevice driver */
	consume_skb(skb);

	if (matches > 0) {
		can_stats.matches++;
		can_stats.matches_delta++;
	}

	return NET_RX_SUCCESS;

drop:
	kfree_skb(skb);
	return NET_RX_DROP;
}


int can_proto_register(struct can_proto *cp)
{
	int proto = cp->protocol;
	int err = 0;

	if (proto < 0 || proto >= CAN_NPROTO) {
		printk(KERN_ERR "can: protocol number %d out of range\n",
		       proto);
		return -EINVAL;
	}

	err = proto_register(cp->prot, 0);
	if (err < 0)
		return err;

	spin_lock(&proto_tab_lock);
	if (proto_tab[proto]) {
		printk(KERN_ERR "can: protocol %d already registered\n",
		       proto);
		err = -EBUSY;
	} else {
		proto_tab[proto] = cp;

		/* use generic ioctl function if not defined by module */
		if (!cp->ops->ioctl)
			cp->ops->ioctl = can_ioctl;
	}
	spin_unlock(&proto_tab_lock);

	if (err < 0)
		proto_unregister(cp->prot);

	return err;
}
EXPORT_SYMBOL(can_proto_register);

void can_proto_unregister(struct can_proto *cp)
{
	int proto = cp->protocol;

	spin_lock(&proto_tab_lock);
	if (!proto_tab[proto]) {
		printk(KERN_ERR "BUG: can: protocol %d is not registered\n",
		       proto);
	}
	proto_tab[proto] = NULL;
	spin_unlock(&proto_tab_lock);

	proto_unregister(cp->prot);
}
EXPORT_SYMBOL(can_proto_unregister);

static int can_notifier(struct notifier_block *nb, unsigned long msg,
			void *data)
{
	struct net_device *dev = (struct net_device *)data;
	struct dev_rcv_lists *d;

	if (!net_eq(dev_net(dev), &init_net))
		return NOTIFY_DONE;

	if (dev->type != ARPHRD_CAN)
		return NOTIFY_DONE;

	switch (msg) {

	case NETDEV_REGISTER:

		/* create new dev_rcv_lists for this device */
		d = kzalloc(sizeof(*d), GFP_KERNEL);
		if (!d) {
			printk(KERN_ERR
			       "can: allocation of receive list failed\n");
			return NOTIFY_DONE;
		}
		BUG_ON(dev->ml_priv);
		dev->ml_priv = d;

		break;

	case NETDEV_UNREGISTER:
		spin_lock(&can_rcvlists_lock);

		d = dev->ml_priv;
		if (d) {
			if (d->entries)
				d->remove_on_zero_entries = 1;
			else {
				kfree(d);
				dev->ml_priv = NULL;
			}
		} else
			printk(KERN_ERR "can: notifier: receive list not "
			       "found for dev %s\n", dev->name);

		spin_unlock(&can_rcvlists_lock);

		break;
	}

	return NOTIFY_DONE;
}


static struct packet_type can_packet __read_mostly = {
	.type = cpu_to_be16(ETH_P_CAN),
	.dev  = NULL,
	.func = can_rcv,
};

static const struct net_proto_family can_family_ops = {
	.family = PF_CAN,
	.create = can_create,
	.owner  = THIS_MODULE,
};

/* notifier block for netdevice event */
static struct notifier_block can_netdev_notifier __read_mostly = {
	.notifier_call = can_notifier,
};

static __init int can_init(void)
{
	printk(banner);

	memset(&can_rx_alldev_list, 0, sizeof(can_rx_alldev_list));

	rcv_cache = kmem_cache_create("can_receiver", sizeof(struct receiver),
				      0, 0, NULL);
	if (!rcv_cache)
		return -ENOMEM;

	if (stats_timer) {
		/* the statistics are updated every second (timer triggered) */
		setup_timer(&can_stattimer, can_stat_update, 0);
		mod_timer(&can_stattimer, round_jiffies(jiffies + HZ));
	} else
		can_stattimer.function = NULL;

	can_init_proc();

	/* protocol register */
	sock_register(&can_family_ops);
	register_netdevice_notifier(&can_netdev_notifier);
	dev_add_pack(&can_packet);

	return 0;
}

static __exit void can_exit(void)
{
	struct net_device *dev;

	if (stats_timer)
		del_timer(&can_stattimer);

	can_remove_proc();

	/* protocol unregister */
	dev_remove_pack(&can_packet);
	unregister_netdevice_notifier(&can_netdev_notifier);
	sock_unregister(PF_CAN);

	/* remove created dev_rcv_lists from still registered CAN devices */
	rcu_read_lock();
	for_each_netdev_rcu(&init_net, dev) {
		if (dev->type == ARPHRD_CAN && dev->ml_priv){

			struct dev_rcv_lists *d = dev->ml_priv;

			BUG_ON(d->entries);
			kfree(d);
			dev->ml_priv = NULL;
		}
	}
	rcu_read_unlock();

	rcu_barrier(); /* Wait for completion of call_rcu()'s */

	kmem_cache_destroy(rcv_cache);
}

module_init(can_init);
module_exit(can_exit);
