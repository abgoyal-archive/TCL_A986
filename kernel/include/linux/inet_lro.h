

#ifndef __INET_LRO_H_
#define __INET_LRO_H_

#include <net/ip.h>
#include <net/tcp.h>


struct net_lro_stats {
	unsigned long aggregated;
	unsigned long flushed;
	unsigned long no_desc;
};

struct net_lro_desc {
	struct sk_buff *parent;
	struct sk_buff *last_skb;
	struct skb_frag_struct *next_frag;
	struct iphdr *iph;
	struct tcphdr *tcph;
	struct vlan_group *vgrp;
	__wsum  data_csum;
	__be32 tcp_rcv_tsecr;
	__be32 tcp_rcv_tsval;
	__be32 tcp_ack;
	u32 tcp_next_seq;
	u32 skb_tot_frags_len;
	u16 ip_tot_len;
	u16 tcp_saw_tstamp; 		/* timestamps enabled */
	__be16 tcp_window;
	u16 vlan_tag;
	int pkt_aggr_cnt;		/* counts aggregated packets */
	int vlan_packet;
	int mss;
	int active;
};


struct net_lro_mgr {
	struct net_device *dev;
	struct net_lro_stats stats;

	/* LRO features */
	unsigned long features;
#define LRO_F_NAPI            1  /* Pass packets to stack via NAPI */
#define LRO_F_EXTRACT_VLAN_ID 2  /* Set flag if VLAN IDs are extracted
				    from received packets and eth protocol
				    is still ETH_P_8021Q */

	/*
	 * Set for generated SKBs that are not added to
	 * the frag list in fragmented mode
	 */
	u32 ip_summed;
	u32 ip_summed_aggr; /* Set in aggregated SKBs: CHECKSUM_UNNECESSARY
			     * or CHECKSUM_NONE */

	int max_desc; /* Max number of LRO descriptors  */
	int max_aggr; /* Max number of LRO packets to be aggregated */

	int frag_align_pad; /* Padding required to properly align layer 3
			     * headers in generated skb when using frags */

	struct net_lro_desc *lro_arr; /* Array of LRO descriptors */

	/*
	 * Optimized driver functions
	 *
	 * get_skb_header: returns tcp and ip header for packet in SKB
	 */
	int (*get_skb_header)(struct sk_buff *skb, void **ip_hdr,
			      void **tcpudp_hdr, u64 *hdr_flags, void *priv);

	/* hdr_flags: */
#define LRO_IPV4 1 /* ip_hdr is IPv4 header */
#define LRO_TCP  2 /* tcpudp_hdr is TCP header */

	/*
	 * get_frag_header: returns mac, tcp and ip header for packet in SKB
	 *
	 * @hdr_flags: Indicate what kind of LRO has to be done
	 *             (IPv4/IPv6/TCP/UDP)
	 */
	int (*get_frag_header)(struct skb_frag_struct *frag, void **mac_hdr,
			       void **ip_hdr, void **tcpudp_hdr, u64 *hdr_flags,
			       void *priv);
};


void lro_receive_skb(struct net_lro_mgr *lro_mgr,
		     struct sk_buff *skb,
		     void *priv);


void lro_vlan_hwaccel_receive_skb(struct net_lro_mgr *lro_mgr,
				  struct sk_buff *skb,
				  struct vlan_group *vgrp,
				  u16 vlan_tag,
				  void *priv);


void lro_receive_frags(struct net_lro_mgr *lro_mgr,
		       struct skb_frag_struct *frags,
		       int len, int true_size, void *priv, __wsum sum);

void lro_vlan_hwaccel_receive_frags(struct net_lro_mgr *lro_mgr,
				    struct skb_frag_struct *frags,
				    int len, int true_size,
				    struct vlan_group *vgrp,
				    u16 vlan_tag,
				    void *priv, __wsum sum);


void lro_flush_all(struct net_lro_mgr *lro_mgr);

void lro_flush_pkt(struct net_lro_mgr *lro_mgr,
		   struct iphdr *iph, struct tcphdr *tcph);

#endif
