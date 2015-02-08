
#ifndef _LINUX_FDDIDEVICE_H
#define _LINUX_FDDIDEVICE_H

#include <linux/if_fddi.h>

#ifdef __KERNEL__
extern __be16	fddi_type_trans(struct sk_buff *skb,
				struct net_device *dev);
extern int fddi_change_mtu(struct net_device *dev, int new_mtu);
extern struct net_device *alloc_fddidev(int sizeof_priv);
#endif

#endif	/* _LINUX_FDDIDEVICE_H */
