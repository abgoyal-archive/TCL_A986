
#ifndef _LINUX_VIRTIO_NET_H
#define _LINUX_VIRTIO_NET_H
#include <linux/types.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_config.h>
#include <linux/if_ether.h>

/* The feature bitmap for virtio net */
#define VIRTIO_NET_F_CSUM	0	/* Host handles pkts w/ partial csum */
#define VIRTIO_NET_F_GUEST_CSUM	1	/* Guest handles pkts w/ partial csum */
#define VIRTIO_NET_F_MAC	5	/* Host has given MAC address. */
#define VIRTIO_NET_F_GSO	6	/* Host handles pkts w/ any GSO type */
#define VIRTIO_NET_F_GUEST_TSO4	7	/* Guest can handle TSOv4 in. */
#define VIRTIO_NET_F_GUEST_TSO6	8	/* Guest can handle TSOv6 in. */
#define VIRTIO_NET_F_GUEST_ECN	9	/* Guest can handle TSO[6] w/ ECN in. */
#define VIRTIO_NET_F_GUEST_UFO	10	/* Guest can handle UFO in. */
#define VIRTIO_NET_F_HOST_TSO4	11	/* Host can handle TSOv4 in. */
#define VIRTIO_NET_F_HOST_TSO6	12	/* Host can handle TSOv6 in. */
#define VIRTIO_NET_F_HOST_ECN	13	/* Host can handle TSO[6] w/ ECN in. */
#define VIRTIO_NET_F_HOST_UFO	14	/* Host can handle UFO in. */
#define VIRTIO_NET_F_MRG_RXBUF	15	/* Host can merge receive buffers. */
#define VIRTIO_NET_F_STATUS	16	/* virtio_net_config.status available */
#define VIRTIO_NET_F_CTRL_VQ	17	/* Control channel available */
#define VIRTIO_NET_F_CTRL_RX	18	/* Control channel RX mode support */
#define VIRTIO_NET_F_CTRL_VLAN	19	/* Control channel VLAN filtering */
#define VIRTIO_NET_F_CTRL_RX_EXTRA 20	/* Extra RX mode control support */

#define VIRTIO_NET_S_LINK_UP	1	/* Link is up */

struct virtio_net_config {
	/* The config defining mac address (if VIRTIO_NET_F_MAC) */
	__u8 mac[6];
	/* See VIRTIO_NET_F_STATUS and VIRTIO_NET_S_* above */
	__u16 status;
} __attribute__((packed));

struct virtio_net_hdr {
#define VIRTIO_NET_HDR_F_NEEDS_CSUM	1	// Use csum_start, csum_offset
	__u8 flags;
#define VIRTIO_NET_HDR_GSO_NONE		0	// Not a GSO frame
#define VIRTIO_NET_HDR_GSO_TCPV4	1	// GSO frame, IPv4 TCP (TSO)
#define VIRTIO_NET_HDR_GSO_UDP		3	// GSO frame, IPv4 UDP (UFO)
#define VIRTIO_NET_HDR_GSO_TCPV6	4	// GSO frame, IPv6 TCP
#define VIRTIO_NET_HDR_GSO_ECN		0x80	// TCP has ECN set
	__u8 gso_type;
	__u16 hdr_len;		/* Ethernet + IP + tcp/udp hdrs */
	__u16 gso_size;		/* Bytes to append to hdr_len per frame */
	__u16 csum_start;	/* Position to start checksumming from */
	__u16 csum_offset;	/* Offset after that to place checksum */
};

struct virtio_net_hdr_mrg_rxbuf {
	struct virtio_net_hdr hdr;
	__u16 num_buffers;	/* Number of merged rx buffers */
};

struct virtio_net_ctrl_hdr {
	__u8 class;
	__u8 cmd;
} __attribute__((packed));

typedef __u8 virtio_net_ctrl_ack;

#define VIRTIO_NET_OK     0
#define VIRTIO_NET_ERR    1

#define VIRTIO_NET_CTRL_RX    0
 #define VIRTIO_NET_CTRL_RX_PROMISC      0
 #define VIRTIO_NET_CTRL_RX_ALLMULTI     1
 #define VIRTIO_NET_CTRL_RX_ALLUNI       2
 #define VIRTIO_NET_CTRL_RX_NOMULTI      3
 #define VIRTIO_NET_CTRL_RX_NOUNI        4
 #define VIRTIO_NET_CTRL_RX_NOBCAST      5

struct virtio_net_ctrl_mac {
	__u32 entries;
	__u8 macs[][ETH_ALEN];
} __attribute__((packed));

#define VIRTIO_NET_CTRL_MAC    1
 #define VIRTIO_NET_CTRL_MAC_TABLE_SET        0

#define VIRTIO_NET_CTRL_VLAN       2
 #define VIRTIO_NET_CTRL_VLAN_ADD             0
 #define VIRTIO_NET_CTRL_VLAN_DEL             1

#endif /* _LINUX_VIRTIO_NET_H */
