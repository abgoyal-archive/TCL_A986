#ifndef _LINUX_ETHERDEVICE_H
#define _LINUX_ETHERDEVICE_H

#include <linux/if_ether.h>
#include <linux/netdevice.h>
#include <linux/random.h>

#ifdef __KERNEL__
extern int		eth_header(struct sk_buff *skb, struct net_device *dev,
				   unsigned short type, void *daddr,
				   void *saddr, unsigned len);
extern int		eth_rebuild_header(struct sk_buff *skb);
extern __be16		eth_type_trans(struct sk_buff *skb, struct net_device *dev);
extern void		eth_header_cache_update(struct hh_cache *hh, struct net_device *dev,
						unsigned char * haddr);
extern int		eth_header_cache(struct neighbour *neigh,
					 struct hh_cache *hh);

extern struct net_device *alloc_etherdev(int sizeof_priv);
static inline void eth_copy_and_sum (struct sk_buff *dest, 
				     const unsigned char *src, 
				     int len, int base)
{
	memcpy (dest->data, src, len);
}

static inline int is_zero_ether_addr(const u8 *addr)
{
	return !(addr[0] | addr[1] | addr[2] | addr[3] | addr[4] | addr[5]);
}

static inline int is_multicast_ether_addr(const u8 *addr)
{
	return (0x01 & addr[0]);
}

static inline int is_broadcast_ether_addr(const u8 *addr)
{
	return (addr[0] & addr[1] & addr[2] & addr[3] & addr[4] & addr[5]) == 0xff;
}

static inline int is_valid_ether_addr(const u8 *addr)
{
	/* FF:FF:FF:FF:FF:FF is a multicast address so we don't need to
	 * explicitly check for it here. */
	return !is_multicast_ether_addr(addr) && !is_zero_ether_addr(addr);
}

static inline void random_ether_addr(u8 *addr)
{
	get_random_bytes (addr, ETH_ALEN);
	addr [0] &= 0xfe;	/* clear multicast bit */
	addr [0] |= 0x02;	/* set local assignment bit (IEEE802) */
}

static inline unsigned compare_ether_addr(const u8 *addr1, const u8 *addr2)
{
	const u16 *a = (const u16 *) addr1;
	const u16 *b = (const u16 *) addr2;

	BUILD_BUG_ON(ETH_ALEN != 6);
	return ((a[0] ^ b[0]) | (a[1] ^ b[1]) | (a[2] ^ b[2])) != 0;
}
#endif	/* __KERNEL__ */

#endif	/* _LINUX_ETHERDEVICE_H */
