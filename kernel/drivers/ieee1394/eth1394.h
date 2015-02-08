

#ifndef __ETH1394_H
#define __ETH1394_H

#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <asm/byteorder.h>

#include "ieee1394.h"
#include "ieee1394_types.h"

#define ETHER1394_REGION_ADDR_LEN	4096

/* GASP identifier numbers for IPv4 over IEEE 1394 */
#define ETHER1394_GASP_SPECIFIER_ID	0x00005E
#define ETHER1394_GASP_SPECIFIER_ID_HI	((0x00005E >> 8) & 0xffff)
#define ETHER1394_GASP_SPECIFIER_ID_LO	(0x00005E & 0xff)
#define ETHER1394_GASP_VERSION		1

#define ETHER1394_GASP_OVERHEAD	(2 * sizeof(quadlet_t))	/* for GASP header */

#define ETHER1394_GASP_BUFFERS	16

#define NODE_SET		(ALL_NODES + 1)		/* Node set == 64 */

enum eth1394_bc_states { ETHER1394_BC_ERROR,
			 ETHER1394_BC_RUNNING,
			 ETHER1394_BC_STOPPED };


/* Private structure for our ethernet driver */
struct eth1394_priv {
	struct hpsb_host *host;		/* The card for this dev	 */
	u16 bc_maxpayload;		/* Max broadcast payload	 */
	u8 bc_sspd;			/* Max broadcast speed		 */
	u64 local_fifo;			/* Local FIFO Address		 */
	spinlock_t lock;		/* Private lock			 */
	int broadcast_channel;		/* Async stream Broadcast Channel */
	enum eth1394_bc_states bc_state; /* broadcast channel state	 */
	struct hpsb_iso *iso;		/* Async stream recv handle	 */
	int bc_dgl;			/* Outgoing broadcast datagram label */
	struct list_head ip_node_list;	/* List of IP capable nodes	 */
	struct unit_directory *ud_list[ALL_NODES]; /* Cached unit dir list */

	struct work_struct wake;	/* Wake up after xmit failure	 */
	struct net_device *wake_dev;	/* Stupid backlink for .wake	 */
	nodeid_t wake_node;		/* Destination of failed xmit	 */
};


#define ETH1394_ALEN (8)
#define ETH1394_HLEN (10)

struct eth1394hdr {
	unsigned char	h_dest[ETH1394_ALEN];	/* destination eth1394 addr	*/
	__be16		h_proto;		/* packet type ID field	*/
}  __attribute__((packed));

static inline struct eth1394hdr *eth1394_hdr(const struct sk_buff *skb)
{
	return (struct eth1394hdr *)skb_mac_header(skb);
}

typedef enum {ETH1394_GASP, ETH1394_WRREQ} eth1394_tx_type;

/* IP1394 headers */

/* Unfragmented */
#if defined __BIG_ENDIAN_BITFIELD
struct eth1394_uf_hdr {
	u16 lf:2;
	u16 res:14;
	__be16 ether_type;		/* Ethernet packet type */
} __attribute__((packed));
#elif defined __LITTLE_ENDIAN_BITFIELD
struct eth1394_uf_hdr {
	u16 res:14;
	u16 lf:2;
	__be16 ether_type;
} __attribute__((packed));
#else
#error Unknown bit field type
#endif

/* First fragment */
#if defined __BIG_ENDIAN_BITFIELD
struct eth1394_ff_hdr {
	u16 lf:2;
	u16 res1:2;
	u16 dg_size:12;		/* Datagram size */
	__be16 ether_type;		/* Ethernet packet type */
	u16 dgl;		/* Datagram label */
	u16 res2;
} __attribute__((packed));
#elif defined __LITTLE_ENDIAN_BITFIELD
struct eth1394_ff_hdr {
	u16 dg_size:12;
	u16 res1:2;
	u16 lf:2;
	__be16 ether_type;
	u16 dgl;
	u16 res2;
} __attribute__((packed));
#else
#error Unknown bit field type
#endif

/* XXX: Subsequent fragments, including last */
#if defined __BIG_ENDIAN_BITFIELD
struct eth1394_sf_hdr {
	u16 lf:2;
	u16 res1:2;
	u16 dg_size:12;		/* Datagram size */
	u16 res2:4;
	u16 fg_off:12;		/* Fragment offset */
	u16 dgl;		/* Datagram label */
	u16 res3;
} __attribute__((packed));
#elif defined __LITTLE_ENDIAN_BITFIELD
struct eth1394_sf_hdr {
	u16 dg_size:12;
	u16 res1:2;
	u16 lf:2;
	u16 fg_off:12;
	u16 res2:4;
	u16 dgl;
	u16 res3;
} __attribute__((packed));
#else
#error Unknown bit field type
#endif

#if defined __BIG_ENDIAN_BITFIELD
struct eth1394_common_hdr {
	u16 lf:2;
	u16 pad1:14;
} __attribute__((packed));
#elif defined __LITTLE_ENDIAN_BITFIELD
struct eth1394_common_hdr {
	u16 pad1:14;
	u16 lf:2;
} __attribute__((packed));
#else
#error Unknown bit field type
#endif

struct eth1394_hdr_words {
	u16 word1;
	u16 word2;
	u16 word3;
	u16 word4;
};

union eth1394_hdr {
	struct eth1394_common_hdr common;
	struct eth1394_uf_hdr uf;
	struct eth1394_ff_hdr ff;
	struct eth1394_sf_hdr sf;
	struct eth1394_hdr_words words;
};

/* End of IP1394 headers */

/* Fragment types */
#define ETH1394_HDR_LF_UF	0	/* unfragmented		*/
#define ETH1394_HDR_LF_FF	1	/* first fragment	*/
#define ETH1394_HDR_LF_LF	2	/* last fragment	*/
#define ETH1394_HDR_LF_IF	3	/* interior fragment	*/

#define IP1394_HW_ADDR_LEN	16	/* As per RFC		*/

/* Our arp packet (ARPHRD_IEEE1394) */
struct eth1394_arp {
	u16 hw_type;		/* 0x0018	*/
	u16 proto_type;		/* 0x0806	*/
	u8 hw_addr_len;		/* 16 		*/
	u8 ip_addr_len;		/* 4		*/
	u16 opcode;		/* ARP Opcode	*/
	/* Above is exactly the same format as struct arphdr */

	__be64 s_uniq_id;	/* Sender's 64bit EUI		*/
	u8 max_rec;		/* Sender's max packet size		*/
	u8 sspd;		/* Sender's max speed			*/
	__be16 fifo_hi;		/* hi 16bits of sender's FIFO addr	*/
	__be32 fifo_lo;		/* lo 32bits of sender's FIFO addr	*/
	u32 sip;		/* Sender's IP Address			*/
	u32 tip;		/* IP Address of requested hw addr	*/
};

/* Network timeout */
#define ETHER1394_TIMEOUT	100000

/* This is our task struct. It's used for the packet complete callback.  */
struct packet_task {
	struct sk_buff *skb;
	int outstanding_pkts;
	eth1394_tx_type tx_type;
	int max_payload;
	struct hpsb_packet *packet;
	struct eth1394_priv *priv;
	union eth1394_hdr hdr;
	u64 addr;
	u16 dest_node;
};

#endif /* __ETH1394_H */
