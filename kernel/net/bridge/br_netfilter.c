

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/ip.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/if_pppox.h>
#include <linux/ppp_defs.h>
#include <linux/netfilter_bridge.h>
#include <linux/netfilter_ipv4.h>
#include <linux/netfilter_ipv6.h>
#include <linux/netfilter_arp.h>
#include <linux/in_route.h>
#include <linux/inetdevice.h>

#include <net/ip.h>
#include <net/ipv6.h>
#include <net/route.h>

#include <asm/uaccess.h>
#include "br_private.h"
#ifdef CONFIG_SYSCTL
#include <linux/sysctl.h>
#endif

#define skb_origaddr(skb)	 (((struct bridge_skb_cb *) \
				 (skb->nf_bridge->data))->daddr.ipv4)
#define store_orig_dstaddr(skb)	 (skb_origaddr(skb) = ip_hdr(skb)->daddr)
#define dnat_took_place(skb)	 (skb_origaddr(skb) != ip_hdr(skb)->daddr)

#ifdef CONFIG_SYSCTL
static struct ctl_table_header *brnf_sysctl_header;
static int brnf_call_iptables __read_mostly = 1;
static int brnf_call_ip6tables __read_mostly = 1;
static int brnf_call_arptables __read_mostly = 1;
static int brnf_filter_vlan_tagged __read_mostly = 0;
static int brnf_filter_pppoe_tagged __read_mostly = 0;
#else
#define brnf_filter_vlan_tagged 0
#define brnf_filter_pppoe_tagged 0
#endif

static inline __be16 vlan_proto(const struct sk_buff *skb)
{
	return vlan_eth_hdr(skb)->h_vlan_encapsulated_proto;
}

#define IS_VLAN_IP(skb) \
	(skb->protocol == htons(ETH_P_8021Q) && \
	 vlan_proto(skb) == htons(ETH_P_IP) && 	\
	 brnf_filter_vlan_tagged)

#define IS_VLAN_IPV6(skb) \
	(skb->protocol == htons(ETH_P_8021Q) && \
	 vlan_proto(skb) == htons(ETH_P_IPV6) &&\
	 brnf_filter_vlan_tagged)

#define IS_VLAN_ARP(skb) \
	(skb->protocol == htons(ETH_P_8021Q) &&	\
	 vlan_proto(skb) == htons(ETH_P_ARP) &&	\
	 brnf_filter_vlan_tagged)

static inline __be16 pppoe_proto(const struct sk_buff *skb)
{
	return *((__be16 *)(skb_mac_header(skb) + ETH_HLEN +
			    sizeof(struct pppoe_hdr)));
}

#define IS_PPPOE_IP(skb) \
	(skb->protocol == htons(ETH_P_PPP_SES) && \
	 pppoe_proto(skb) == htons(PPP_IP) && \
	 brnf_filter_pppoe_tagged)

#define IS_PPPOE_IPV6(skb) \
	(skb->protocol == htons(ETH_P_PPP_SES) && \
	 pppoe_proto(skb) == htons(PPP_IPV6) && \
	 brnf_filter_pppoe_tagged)

static void fake_update_pmtu(struct dst_entry *dst, u32 mtu)
{
}

static struct dst_ops fake_dst_ops = {
	.family =		AF_INET,
	.protocol =		cpu_to_be16(ETH_P_IP),
	.update_pmtu =		fake_update_pmtu,
	.entries =		ATOMIC_INIT(0),
};

void br_netfilter_rtable_init(struct net_bridge *br)
{
	struct rtable *rt = &br->fake_rtable;

	atomic_set(&rt->u.dst.__refcnt, 1);
	rt->u.dst.dev = br->dev;
	rt->u.dst.path = &rt->u.dst;
	rt->u.dst.metrics[RTAX_MTU - 1] = 1500;
	rt->u.dst.flags	= DST_NOXFRM;
	rt->u.dst.ops = &fake_dst_ops;
}

static inline struct rtable *bridge_parent_rtable(const struct net_device *dev)
{
	struct net_bridge_port *port = rcu_dereference(dev->br_port);

	return port ? &port->br->fake_rtable : NULL;
}

static inline struct net_device *bridge_parent(const struct net_device *dev)
{
	struct net_bridge_port *port = rcu_dereference(dev->br_port);

	return port ? port->br->dev : NULL;
}

static inline struct nf_bridge_info *nf_bridge_alloc(struct sk_buff *skb)
{
	skb->nf_bridge = kzalloc(sizeof(struct nf_bridge_info), GFP_ATOMIC);
	if (likely(skb->nf_bridge))
		atomic_set(&(skb->nf_bridge->use), 1);

	return skb->nf_bridge;
}

static inline struct nf_bridge_info *nf_bridge_unshare(struct sk_buff *skb)
{
	struct nf_bridge_info *nf_bridge = skb->nf_bridge;

	if (atomic_read(&nf_bridge->use) > 1) {
		struct nf_bridge_info *tmp = nf_bridge_alloc(skb);

		if (tmp) {
			memcpy(tmp, nf_bridge, sizeof(struct nf_bridge_info));
			atomic_set(&tmp->use, 1);
			nf_bridge_put(nf_bridge);
		}
		nf_bridge = tmp;
	}
	return nf_bridge;
}

static inline void nf_bridge_push_encap_header(struct sk_buff *skb)
{
	unsigned int len = nf_bridge_encap_header_len(skb);

	skb_push(skb, len);
	skb->network_header -= len;
}

static inline void nf_bridge_pull_encap_header(struct sk_buff *skb)
{
	unsigned int len = nf_bridge_encap_header_len(skb);

	skb_pull(skb, len);
	skb->network_header += len;
}

static inline void nf_bridge_pull_encap_header_rcsum(struct sk_buff *skb)
{
	unsigned int len = nf_bridge_encap_header_len(skb);

	skb_pull_rcsum(skb, len);
	skb->network_header += len;
}

static inline void nf_bridge_save_header(struct sk_buff *skb)
{
	int header_size = ETH_HLEN + nf_bridge_encap_header_len(skb);

	skb_copy_from_linear_data_offset(skb, -header_size,
					 skb->nf_bridge->data, header_size);
}

static inline void nf_bridge_update_protocol(struct sk_buff *skb)
{
	if (skb->nf_bridge->mask & BRNF_8021Q)
		skb->protocol = htons(ETH_P_8021Q);
	else if (skb->nf_bridge->mask & BRNF_PPPoE)
		skb->protocol = htons(ETH_P_PPP_SES);
}

int nf_bridge_copy_header(struct sk_buff *skb)
{
	int err;
	unsigned int header_size;

	nf_bridge_update_protocol(skb);
	header_size = ETH_HLEN + nf_bridge_encap_header_len(skb);
	err = skb_cow_head(skb, header_size);
	if (err)
		return err;

	skb_copy_to_linear_data_offset(skb, -header_size,
				       skb->nf_bridge->data, header_size);
	__skb_push(skb, nf_bridge_encap_header_len(skb));
	return 0;
}

/* PF_BRIDGE/PRE_ROUTING *********************************************/
static int br_nf_pre_routing_finish_ipv6(struct sk_buff *skb)
{
	struct nf_bridge_info *nf_bridge = skb->nf_bridge;
	struct rtable *rt;

	if (nf_bridge->mask & BRNF_PKT_TYPE) {
		skb->pkt_type = PACKET_OTHERHOST;
		nf_bridge->mask ^= BRNF_PKT_TYPE;
	}
	nf_bridge->mask ^= BRNF_NF_BRIDGE_PREROUTING;

	rt = bridge_parent_rtable(nf_bridge->physindev);
	if (!rt) {
		kfree_skb(skb);
		return 0;
	}
	dst_hold(&rt->u.dst);
	skb_dst_set(skb, &rt->u.dst);

	skb->dev = nf_bridge->physindev;
	nf_bridge_update_protocol(skb);
	nf_bridge_push_encap_header(skb);
	NF_HOOK_THRESH(NFPROTO_BRIDGE, NF_BR_PRE_ROUTING, skb, skb->dev, NULL,
		       br_handle_frame_finish, 1);

	return 0;
}

static int br_nf_pre_routing_finish_bridge(struct sk_buff *skb)
{
	struct nf_bridge_info *nf_bridge = skb->nf_bridge;
	struct dst_entry *dst;

	skb->dev = bridge_parent(skb->dev);
	if (!skb->dev)
		goto free_skb;
	dst = skb_dst(skb);
	if (dst->hh) {
		neigh_hh_bridge(dst->hh, skb);
		skb->dev = nf_bridge->physindev;
		return br_handle_frame_finish(skb);
	} else if (dst->neighbour) {
		/* the neighbour function below overwrites the complete
		 * MAC header, so we save the Ethernet source address and
		 * protocol number. */
		skb_copy_from_linear_data_offset(skb, -(ETH_HLEN-ETH_ALEN), skb->nf_bridge->data, ETH_HLEN-ETH_ALEN);
		/* tell br_dev_xmit to continue with forwarding */
		nf_bridge->mask |= BRNF_BRIDGED_DNAT;
		return dst->neighbour->output(skb);
	}
free_skb:
	kfree_skb(skb);
	return 0;
}

static int br_nf_pre_routing_finish(struct sk_buff *skb)
{
	struct net_device *dev = skb->dev;
	struct iphdr *iph = ip_hdr(skb);
	struct nf_bridge_info *nf_bridge = skb->nf_bridge;
	struct rtable *rt;
	int err;

	if (nf_bridge->mask & BRNF_PKT_TYPE) {
		skb->pkt_type = PACKET_OTHERHOST;
		nf_bridge->mask ^= BRNF_PKT_TYPE;
	}
	nf_bridge->mask ^= BRNF_NF_BRIDGE_PREROUTING;
	if (dnat_took_place(skb)) {
		if ((err = ip_route_input(skb, iph->daddr, iph->saddr, iph->tos, dev))) {
			struct flowi fl = {
				.nl_u = {
					.ip4_u = {
						 .daddr = iph->daddr,
						 .saddr = 0,
						 .tos = RT_TOS(iph->tos) },
				},
				.proto = 0,
			};
			struct in_device *in_dev = __in_dev_get_rcu(dev);

			/* If err equals -EHOSTUNREACH the error is due to a
			 * martian destination or due to the fact that
			 * forwarding is disabled. For most martian packets,
			 * ip_route_output_key() will fail. It won't fail for 2 types of
			 * martian destinations: loopback destinations and destination
			 * 0.0.0.0. In both cases the packet will be dropped because the
			 * destination is the loopback device and not the bridge. */
			if (err != -EHOSTUNREACH || !in_dev || IN_DEV_FORWARD(in_dev))
				goto free_skb;

			if (!ip_route_output_key(dev_net(dev), &rt, &fl)) {
				/* - Bridged-and-DNAT'ed traffic doesn't
				 *   require ip_forwarding. */
				if (((struct dst_entry *)rt)->dev == dev) {
					skb_dst_set(skb, (struct dst_entry *)rt);
					goto bridged_dnat;
				}
				dst_release((struct dst_entry *)rt);
			}
free_skb:
			kfree_skb(skb);
			return 0;
		} else {
			if (skb_dst(skb)->dev == dev) {
bridged_dnat:
				skb->dev = nf_bridge->physindev;
				nf_bridge_update_protocol(skb);
				nf_bridge_push_encap_header(skb);
				NF_HOOK_THRESH(NFPROTO_BRIDGE,
					       NF_BR_PRE_ROUTING,
					       skb, skb->dev, NULL,
					       br_nf_pre_routing_finish_bridge,
					       1);
				return 0;
			}
			memcpy(eth_hdr(skb)->h_dest, dev->dev_addr, ETH_ALEN);
			skb->pkt_type = PACKET_HOST;
		}
	} else {
		rt = bridge_parent_rtable(nf_bridge->physindev);
		if (!rt) {
			kfree_skb(skb);
			return 0;
		}
		dst_hold(&rt->u.dst);
		skb_dst_set(skb, &rt->u.dst);
	}

	skb->dev = nf_bridge->physindev;
	nf_bridge_update_protocol(skb);
	nf_bridge_push_encap_header(skb);
	NF_HOOK_THRESH(NFPROTO_BRIDGE, NF_BR_PRE_ROUTING, skb, skb->dev, NULL,
		       br_handle_frame_finish, 1);

	return 0;
}

/* Some common code for IPv4/IPv6 */
static struct net_device *setup_pre_routing(struct sk_buff *skb)
{
	struct nf_bridge_info *nf_bridge = skb->nf_bridge;

	if (skb->pkt_type == PACKET_OTHERHOST) {
		skb->pkt_type = PACKET_HOST;
		nf_bridge->mask |= BRNF_PKT_TYPE;
	}

	nf_bridge->mask |= BRNF_NF_BRIDGE_PREROUTING;
	nf_bridge->physindev = skb->dev;
	skb->dev = bridge_parent(skb->dev);
	if (skb->protocol == htons(ETH_P_8021Q))
		nf_bridge->mask |= BRNF_8021Q;
	else if (skb->protocol == htons(ETH_P_PPP_SES))
		nf_bridge->mask |= BRNF_PPPoE;

	return skb->dev;
}

/* We only check the length. A bridge shouldn't do any hop-by-hop stuff anyway */
static int check_hbh_len(struct sk_buff *skb)
{
	unsigned char *raw = (u8 *)(ipv6_hdr(skb) + 1);
	u32 pkt_len;
	const unsigned char *nh = skb_network_header(skb);
	int off = raw - nh;
	int len = (raw[1] + 1) << 3;

	if ((raw + len) - skb->data > skb_headlen(skb))
		goto bad;

	off += 2;
	len -= 2;

	while (len > 0) {
		int optlen = nh[off + 1] + 2;

		switch (nh[off]) {
		case IPV6_TLV_PAD0:
			optlen = 1;
			break;

		case IPV6_TLV_PADN:
			break;

		case IPV6_TLV_JUMBO:
			if (nh[off + 1] != 4 || (off & 3) != 2)
				goto bad;
			pkt_len = ntohl(*(__be32 *) (nh + off + 2));
			if (pkt_len <= IPV6_MAXPLEN ||
			    ipv6_hdr(skb)->payload_len)
				goto bad;
			if (pkt_len > skb->len - sizeof(struct ipv6hdr))
				goto bad;
			if (pskb_trim_rcsum(skb,
					    pkt_len + sizeof(struct ipv6hdr)))
				goto bad;
			nh = skb_network_header(skb);
			break;
		default:
			if (optlen > len)
				goto bad;
			break;
		}
		off += optlen;
		len -= optlen;
	}
	if (len == 0)
		return 0;
bad:
	return -1;

}

static unsigned int br_nf_pre_routing_ipv6(unsigned int hook,
					   struct sk_buff *skb,
					   const struct net_device *in,
					   const struct net_device *out,
					   int (*okfn)(struct sk_buff *))
{
	struct ipv6hdr *hdr;
	u32 pkt_len;

	if (skb->len < sizeof(struct ipv6hdr))
		goto inhdr_error;

	if (!pskb_may_pull(skb, sizeof(struct ipv6hdr)))
		goto inhdr_error;

	hdr = ipv6_hdr(skb);

	if (hdr->version != 6)
		goto inhdr_error;

	pkt_len = ntohs(hdr->payload_len);

	if (pkt_len || hdr->nexthdr != NEXTHDR_HOP) {
		if (pkt_len + sizeof(struct ipv6hdr) > skb->len)
			goto inhdr_error;
		if (pskb_trim_rcsum(skb, pkt_len + sizeof(struct ipv6hdr)))
			goto inhdr_error;
	}
	if (hdr->nexthdr == NEXTHDR_HOP && check_hbh_len(skb))
		goto inhdr_error;

	nf_bridge_put(skb->nf_bridge);
	if (!nf_bridge_alloc(skb))
		return NF_DROP;
	if (!setup_pre_routing(skb))
		return NF_DROP;

	skb->protocol = htons(ETH_P_IPV6);
	NF_HOOK(NFPROTO_IPV6, NF_INET_PRE_ROUTING, skb, skb->dev, NULL,
		br_nf_pre_routing_finish_ipv6);

	return NF_STOLEN;

inhdr_error:
	return NF_DROP;
}

static unsigned int br_nf_pre_routing(unsigned int hook, struct sk_buff *skb,
				      const struct net_device *in,
				      const struct net_device *out,
				      int (*okfn)(struct sk_buff *))
{
	struct iphdr *iph;
	__u32 len = nf_bridge_encap_header_len(skb);

	if (unlikely(!pskb_may_pull(skb, len)))
		goto out;

	if (skb->protocol == htons(ETH_P_IPV6) || IS_VLAN_IPV6(skb) ||
	    IS_PPPOE_IPV6(skb)) {
#ifdef CONFIG_SYSCTL
		if (!brnf_call_ip6tables)
			return NF_ACCEPT;
#endif
		nf_bridge_pull_encap_header_rcsum(skb);
		return br_nf_pre_routing_ipv6(hook, skb, in, out, okfn);
	}
#ifdef CONFIG_SYSCTL
	if (!brnf_call_iptables)
		return NF_ACCEPT;
#endif

	if (skb->protocol != htons(ETH_P_IP) && !IS_VLAN_IP(skb) &&
	    !IS_PPPOE_IP(skb))
		return NF_ACCEPT;

	nf_bridge_pull_encap_header_rcsum(skb);

	if (!pskb_may_pull(skb, sizeof(struct iphdr)))
		goto inhdr_error;

	iph = ip_hdr(skb);
	if (iph->ihl < 5 || iph->version != 4)
		goto inhdr_error;

	if (!pskb_may_pull(skb, 4 * iph->ihl))
		goto inhdr_error;

	iph = ip_hdr(skb);
	if (ip_fast_csum((__u8 *) iph, iph->ihl) != 0)
		goto inhdr_error;

	len = ntohs(iph->tot_len);
	if (skb->len < len || len < 4 * iph->ihl)
		goto inhdr_error;

	pskb_trim_rcsum(skb, len);

	/* BUG: Should really parse the IP options here. */
	memset(IPCB(skb), 0, sizeof(struct inet_skb_parm));

	nf_bridge_put(skb->nf_bridge);
	if (!nf_bridge_alloc(skb))
		return NF_DROP;
	if (!setup_pre_routing(skb))
		return NF_DROP;
	store_orig_dstaddr(skb);
	skb->protocol = htons(ETH_P_IP);

	NF_HOOK(NFPROTO_IPV4, NF_INET_PRE_ROUTING, skb, skb->dev, NULL,
		br_nf_pre_routing_finish);

	return NF_STOLEN;

inhdr_error:
//      IP_INC_STATS_BH(IpInHdrErrors);
out:
	return NF_DROP;
}


/* PF_BRIDGE/LOCAL_IN ************************************************/
static unsigned int br_nf_local_in(unsigned int hook, struct sk_buff *skb,
				   const struct net_device *in,
				   const struct net_device *out,
				   int (*okfn)(struct sk_buff *))
{
	struct rtable *rt = skb_rtable(skb);

	if (rt && rt == bridge_parent_rtable(in))
		skb_dst_drop(skb);

	return NF_ACCEPT;
}

/* PF_BRIDGE/FORWARD *************************************************/
static int br_nf_forward_finish(struct sk_buff *skb)
{
	struct nf_bridge_info *nf_bridge = skb->nf_bridge;
	struct net_device *in;

	if (skb->protocol != htons(ETH_P_ARP) && !IS_VLAN_ARP(skb)) {
		in = nf_bridge->physindev;
		if (nf_bridge->mask & BRNF_PKT_TYPE) {
			skb->pkt_type = PACKET_OTHERHOST;
			nf_bridge->mask ^= BRNF_PKT_TYPE;
		}
		nf_bridge_update_protocol(skb);
	} else {
		in = *((struct net_device **)(skb->cb));
	}
	nf_bridge_push_encap_header(skb);

	NF_HOOK_THRESH(NFPROTO_BRIDGE, NF_BR_FORWARD, skb, in,
		       skb->dev, br_forward_finish, 1);
	return 0;
}

static unsigned int br_nf_forward_ip(unsigned int hook, struct sk_buff *skb,
				     const struct net_device *in,
				     const struct net_device *out,
				     int (*okfn)(struct sk_buff *))
{
	struct nf_bridge_info *nf_bridge;
	struct net_device *parent;
	u_int8_t pf;

	if (!skb->nf_bridge)
		return NF_ACCEPT;

	/* Need exclusive nf_bridge_info since we might have multiple
	 * different physoutdevs. */
	if (!nf_bridge_unshare(skb))
		return NF_DROP;

	parent = bridge_parent(out);
	if (!parent)
		return NF_DROP;

	if (skb->protocol == htons(ETH_P_IP) || IS_VLAN_IP(skb) ||
	    IS_PPPOE_IP(skb))
		pf = PF_INET;
	else if (skb->protocol == htons(ETH_P_IPV6) || IS_VLAN_IPV6(skb) ||
		 IS_PPPOE_IPV6(skb))
		pf = PF_INET6;
	else
		return NF_ACCEPT;

	nf_bridge_pull_encap_header(skb);

	nf_bridge = skb->nf_bridge;
	if (skb->pkt_type == PACKET_OTHERHOST) {
		skb->pkt_type = PACKET_HOST;
		nf_bridge->mask |= BRNF_PKT_TYPE;
	}

	/* The physdev module checks on this */
	nf_bridge->mask |= BRNF_BRIDGED;
	nf_bridge->physoutdev = skb->dev;
	if (pf == PF_INET)
		skb->protocol = htons(ETH_P_IP);
	else
		skb->protocol = htons(ETH_P_IPV6);

	NF_HOOK(pf, NF_INET_FORWARD, skb, bridge_parent(in), parent,
		br_nf_forward_finish);

	return NF_STOLEN;
}

static unsigned int br_nf_forward_arp(unsigned int hook, struct sk_buff *skb,
				      const struct net_device *in,
				      const struct net_device *out,
				      int (*okfn)(struct sk_buff *))
{
	struct net_device **d = (struct net_device **)(skb->cb);

#ifdef CONFIG_SYSCTL
	if (!brnf_call_arptables)
		return NF_ACCEPT;
#endif

	if (skb->protocol != htons(ETH_P_ARP)) {
		if (!IS_VLAN_ARP(skb))
			return NF_ACCEPT;
		nf_bridge_pull_encap_header(skb);
	}

	if (arp_hdr(skb)->ar_pln != 4) {
		if (IS_VLAN_ARP(skb))
			nf_bridge_push_encap_header(skb);
		return NF_ACCEPT;
	}
	*d = (struct net_device *)in;
	NF_HOOK(NFPROTO_ARP, NF_ARP_FORWARD, skb, (struct net_device *)in,
		(struct net_device *)out, br_nf_forward_finish);

	return NF_STOLEN;
}

#if defined(CONFIG_NF_CONNTRACK_IPV4) || defined(CONFIG_NF_CONNTRACK_IPV4_MODULE)
static int br_nf_dev_queue_xmit(struct sk_buff *skb)
{
	if (skb->nfct != NULL && skb->protocol == htons(ETH_P_IP) &&
	    skb->len + nf_bridge_mtu_reduction(skb) > skb->dev->mtu &&
	    !skb_is_gso(skb)) {
		/* BUG: Should really parse the IP options here. */
		memset(IPCB(skb), 0, sizeof(struct inet_skb_parm));
		return ip_fragment(skb, br_dev_queue_push_xmit);
	} else
		return br_dev_queue_push_xmit(skb);
}
#else
static int br_nf_dev_queue_xmit(struct sk_buff *skb)
{
        return br_dev_queue_push_xmit(skb);
}
#endif

/* PF_BRIDGE/POST_ROUTING ********************************************/
static unsigned int br_nf_post_routing(unsigned int hook, struct sk_buff *skb,
				       const struct net_device *in,
				       const struct net_device *out,
				       int (*okfn)(struct sk_buff *))
{
	struct nf_bridge_info *nf_bridge = skb->nf_bridge;
	struct net_device *realoutdev = bridge_parent(skb->dev);
	u_int8_t pf;

	if (!nf_bridge || !(nf_bridge->mask & BRNF_BRIDGED))
		return NF_ACCEPT;

	if (!realoutdev)
		return NF_DROP;

	if (skb->protocol == htons(ETH_P_IP) || IS_VLAN_IP(skb) ||
	    IS_PPPOE_IP(skb))
		pf = PF_INET;
	else if (skb->protocol == htons(ETH_P_IPV6) || IS_VLAN_IPV6(skb) ||
		 IS_PPPOE_IPV6(skb))
		pf = PF_INET6;
	else
		return NF_ACCEPT;

	/* We assume any code from br_dev_queue_push_xmit onwards doesn't care
	 * about the value of skb->pkt_type. */
	if (skb->pkt_type == PACKET_OTHERHOST) {
		skb->pkt_type = PACKET_HOST;
		nf_bridge->mask |= BRNF_PKT_TYPE;
	}

	nf_bridge_pull_encap_header(skb);
	nf_bridge_save_header(skb);
	if (pf == PF_INET)
		skb->protocol = htons(ETH_P_IP);
	else
		skb->protocol = htons(ETH_P_IPV6);

	NF_HOOK(pf, NF_INET_POST_ROUTING, skb, NULL, realoutdev,
		br_nf_dev_queue_xmit);

	return NF_STOLEN;
}

/* IP/SABOTAGE *****************************************************/
static unsigned int ip_sabotage_in(unsigned int hook, struct sk_buff *skb,
				   const struct net_device *in,
				   const struct net_device *out,
				   int (*okfn)(struct sk_buff *))
{
	if (skb->nf_bridge &&
	    !(skb->nf_bridge->mask & BRNF_NF_BRIDGE_PREROUTING)) {
		return NF_STOP;
	}

	return NF_ACCEPT;
}

static struct nf_hook_ops br_nf_ops[] __read_mostly = {
	{
		.hook = br_nf_pre_routing,
		.owner = THIS_MODULE,
		.pf = PF_BRIDGE,
		.hooknum = NF_BR_PRE_ROUTING,
		.priority = NF_BR_PRI_BRNF,
	},
	{
		.hook = br_nf_local_in,
		.owner = THIS_MODULE,
		.pf = PF_BRIDGE,
		.hooknum = NF_BR_LOCAL_IN,
		.priority = NF_BR_PRI_BRNF,
	},
	{
		.hook = br_nf_forward_ip,
		.owner = THIS_MODULE,
		.pf = PF_BRIDGE,
		.hooknum = NF_BR_FORWARD,
		.priority = NF_BR_PRI_BRNF - 1,
	},
	{
		.hook = br_nf_forward_arp,
		.owner = THIS_MODULE,
		.pf = PF_BRIDGE,
		.hooknum = NF_BR_FORWARD,
		.priority = NF_BR_PRI_BRNF,
	},
	{
		.hook = br_nf_post_routing,
		.owner = THIS_MODULE,
		.pf = PF_BRIDGE,
		.hooknum = NF_BR_POST_ROUTING,
		.priority = NF_BR_PRI_LAST,
	},
	{
		.hook = ip_sabotage_in,
		.owner = THIS_MODULE,
		.pf = PF_INET,
		.hooknum = NF_INET_PRE_ROUTING,
		.priority = NF_IP_PRI_FIRST,
	},
	{
		.hook = ip_sabotage_in,
		.owner = THIS_MODULE,
		.pf = PF_INET6,
		.hooknum = NF_INET_PRE_ROUTING,
		.priority = NF_IP6_PRI_FIRST,
	},
};

#ifdef CONFIG_SYSCTL
static
int brnf_sysctl_call_tables(ctl_table * ctl, int write,
			    void __user * buffer, size_t * lenp, loff_t * ppos)
{
	int ret;

	ret = proc_dointvec(ctl, write, buffer, lenp, ppos);

	if (write && *(int *)(ctl->data))
		*(int *)(ctl->data) = 1;
	return ret;
}

static ctl_table brnf_table[] = {
	{
		.procname	= "bridge-nf-call-arptables",
		.data		= &brnf_call_arptables,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= brnf_sysctl_call_tables,
	},
	{
		.procname	= "bridge-nf-call-iptables",
		.data		= &brnf_call_iptables,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= brnf_sysctl_call_tables,
	},
	{
		.procname	= "bridge-nf-call-ip6tables",
		.data		= &brnf_call_ip6tables,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= brnf_sysctl_call_tables,
	},
	{
		.procname	= "bridge-nf-filter-vlan-tagged",
		.data		= &brnf_filter_vlan_tagged,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= brnf_sysctl_call_tables,
	},
	{
		.procname	= "bridge-nf-filter-pppoe-tagged",
		.data		= &brnf_filter_pppoe_tagged,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= brnf_sysctl_call_tables,
	},
	{ }
};

static struct ctl_path brnf_path[] = {
	{ .procname = "net", },
	{ .procname = "bridge", },
	{ }
};
#endif

int __init br_netfilter_init(void)
{
	int ret;

	ret = nf_register_hooks(br_nf_ops, ARRAY_SIZE(br_nf_ops));
	if (ret < 0)
		return ret;
#ifdef CONFIG_SYSCTL
	brnf_sysctl_header = register_sysctl_paths(brnf_path, brnf_table);
	if (brnf_sysctl_header == NULL) {
		printk(KERN_WARNING
		       "br_netfilter: can't register to sysctl.\n");
		nf_unregister_hooks(br_nf_ops, ARRAY_SIZE(br_nf_ops));
		return -ENOMEM;
	}
#endif
	printk(KERN_NOTICE "Bridge firewalling registered\n");
	return 0;
}

void br_netfilter_fini(void)
{
	nf_unregister_hooks(br_nf_ops, ARRAY_SIZE(br_nf_ops));
#ifdef CONFIG_SYSCTL
	unregister_sysctl_table(brnf_sysctl_header);
#endif
}
