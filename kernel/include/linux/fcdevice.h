
#ifndef _LINUX_FCDEVICE_H
#define _LINUX_FCDEVICE_H


#include <linux/if_fc.h>

#ifdef __KERNEL__
extern struct net_device *alloc_fcdev(int sizeof_priv);
#endif

#endif	/* _LINUX_FCDEVICE_H */
