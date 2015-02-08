

#include <linux/types.h>
#include <linux/ip.h>
#include <linux/netfilter.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/icmp.h>
#include <linux/sysctl.h>
#include <net/ip.h>

#include <linux/netfilter_ipv4.h>
#include <net/netfilter/nf_conntrack.h>
#include <net/netfilter/nf_conntrack_l4proto.h>
#include <net/netfilter/nf_conntrack_l3proto.h>
#include <net/netfilter/nf_conntrack_core.h>
#include <net/netfilter/ipv4/nf_conntrack_ipv4.h>

static bool generic_pkt_to_tuple(const struct sk_buff *skb, unsigned int nhoff,
				 struct nf_conntrack_tuple *tuple)
{
	memset(&tuple->src.u3, 0, sizeof(tuple->src.u3));
	memset(&tuple->dst.u3, 0, sizeof(tuple->dst.u3));

	return true;
}

static bool generic_invert_tuple(struct nf_conntrack_tuple *tuple,
				 const struct nf_conntrack_tuple *orig)
{
	memset(&tuple->src.u3, 0, sizeof(tuple->src.u3));
	memset(&tuple->dst.u3, 0, sizeof(tuple->dst.u3));

	return true;
}

static int generic_print_tuple(struct seq_file *s,
			    const struct nf_conntrack_tuple *tuple)
{
	return 0;
}

static int generic_get_l4proto(const struct sk_buff *skb, unsigned int nhoff,
			       unsigned int *dataoff, u_int8_t *protonum)
{
	/* Never track !!! */
	return -NF_ACCEPT;
}


struct nf_conntrack_l3proto nf_conntrack_l3proto_generic __read_mostly = {
	.l3proto	 = PF_UNSPEC,
	.name		 = "unknown",
	.pkt_to_tuple	 = generic_pkt_to_tuple,
	.invert_tuple	 = generic_invert_tuple,
	.print_tuple	 = generic_print_tuple,
	.get_l4proto	 = generic_get_l4proto,
};
EXPORT_SYMBOL_GPL(nf_conntrack_l3proto_generic);
