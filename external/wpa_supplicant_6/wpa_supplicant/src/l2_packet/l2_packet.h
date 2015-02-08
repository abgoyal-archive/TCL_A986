

#ifndef L2_PACKET_H
#define L2_PACKET_H

struct l2_packet_data;

#ifdef _MSC_VER
#pragma pack(push, 1)
#endif /* _MSC_VER */

struct l2_ethhdr {
	u8 h_dest[ETH_ALEN];
	u8 h_source[ETH_ALEN];
	be16 h_proto;
} STRUCT_PACKED;

#ifdef _MSC_VER
#pragma pack(pop)
#endif /* _MSC_VER */

struct l2_packet_data * l2_packet_init(
	const char *ifname, const u8 *own_addr, unsigned short protocol,
	void (*rx_callback)(void *ctx, const u8 *src_addr,
			    const u8 *buf, size_t len),
	void *rx_callback_ctx, int l2_hdr);

void l2_packet_deinit(struct l2_packet_data *l2);

int l2_packet_get_own_addr(struct l2_packet_data *l2, u8 *addr);

int l2_packet_send(struct l2_packet_data *l2, const u8 *dst_addr, u16 proto,
		   const u8 *buf, size_t len);

int l2_packet_get_ip_addr(struct l2_packet_data *l2, char *buf, size_t len);


void l2_packet_notify_auth_start(struct l2_packet_data *l2);

#endif /* L2_PACKET_H */
