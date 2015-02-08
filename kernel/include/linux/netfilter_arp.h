
#ifndef __LINUX_ARP_NETFILTER_H
#define __LINUX_ARP_NETFILTER_H


#include <linux/netfilter.h>

/* There is no PF_ARP. */
#define NF_ARP		0

/* ARP Hooks */
#define NF_ARP_IN	0
#define NF_ARP_OUT	1
#define NF_ARP_FORWARD	2
#define NF_ARP_NUMHOOKS	3

#endif /* __LINUX_ARP_NETFILTER_H */
