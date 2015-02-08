
#ifndef _LINUX_ETHERDEVICE_H
#define _LINUX_ETHERDEVICE_H

#include <linux/if_ether.h>
#include <linux/netdevice.h>
#include <linux/random.h>
#include <asm/unaligned.h>

#ifdef __KERNEL__
extern __be16		eth_type_trans(struct sk_buff *skb, struct net_device *dev);
extern const struct header_ops eth_header_ops;

extern int eth_header(struct sk_buff *skb, struct net_device *dev,
		      unsigned short type,
		      const void *daddr, const void *saddr, unsigned len);
extern int eth_rebuild_header(struct sk_buff *skb);
extern int eth_header_parse(const struct sk_buff *skb, unsigned char *haddr);
extern int eth_header_cache(const struct neighbour *neigh, struct hh_cache *hh);
extern void eth_header_cache_update(struct hh_cache *hh,
				    const struct net_device *dev,
				    const unsigned char *haddr);
extern int eth_mac_addr(struct net_device *dev, void *p);
extern int eth_change_mtu(struct net_device *dev, int new_mtu);
extern int eth_validate_addr(struct net_device *dev);



extern struct net_device *alloc_etherdev_mq(int sizeof_priv, unsigned int queue_count);
#define alloc_etherdev(sizeof_priv) alloc_etherdev_mq(sizeof_priv, 1)

static inline int is_zero_ether_addr(const u8 *addr)
{
	return !(addr[0] | addr[1] | addr[2] | addr[3] | addr[4] | addr[5]);
}

static inline int is_multicast_ether_addr(const u8 *addr)
{
	return (0x01 & addr[0]);
}

static inline int is_local_ether_addr(const u8 *addr)
{
	return (0x02 & addr[0]);
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

static inline unsigned long zap_last_2bytes(unsigned long value)
{
#ifdef __BIG_ENDIAN
	return value >> 16;
#else
	return value << 16;
#endif
}


static inline unsigned compare_ether_addr_64bits(const u8 addr1[6+2],
						 const u8 addr2[6+2])
{
#ifdef CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS
	unsigned long fold = ((*(unsigned long *)addr1) ^
			      (*(unsigned long *)addr2));

	if (sizeof(fold) == 8)
		return zap_last_2bytes(fold) != 0;

	fold |= zap_last_2bytes((*(unsigned long *)(addr1 + 4)) ^
				(*(unsigned long *)(addr2 + 4)));
	return fold != 0;
#else
	return compare_ether_addr(addr1, addr2);
#endif
}

static inline bool is_etherdev_addr(const struct net_device *dev,
				    const u8 addr[6 + 2])
{
	struct netdev_hw_addr *ha;
	int res = 1;

	rcu_read_lock();
	for_each_dev_addr(dev, ha) {
		res = compare_ether_addr_64bits(addr, ha->addr);
		if (!res)
			break;
	}
	rcu_read_unlock();
	return !res;
}
#endif	/* __KERNEL__ */


static inline int compare_ether_header(const void *a, const void *b)
{
	u32 *a32 = (u32 *)((u8 *)a + 2);
	u32 *b32 = (u32 *)((u8 *)b + 2);

	return (*(u16 *)a ^ *(u16 *)b) | (a32[0] ^ b32[0]) |
	       (a32[1] ^ b32[1]) | (a32[2] ^ b32[2]);
}

#endif	/* _LINUX_ETHERDEVICE_H */
