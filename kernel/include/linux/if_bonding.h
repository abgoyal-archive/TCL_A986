

#ifndef _LINUX_IF_BONDING_H
#define _LINUX_IF_BONDING_H

#include <linux/if.h>
#include <linux/types.h>
#include <linux/if_ether.h>

/* userland - kernel ABI version (2003/05/08) */
#define BOND_ABI_VERSION 2

#define BOND_ENSLAVE_OLD		(SIOCDEVPRIVATE)
#define BOND_RELEASE_OLD		(SIOCDEVPRIVATE + 1)
#define BOND_SETHWADDR_OLD		(SIOCDEVPRIVATE + 2)
#define BOND_SLAVE_INFO_QUERY_OLD	(SIOCDEVPRIVATE + 11)
#define BOND_INFO_QUERY_OLD		(SIOCDEVPRIVATE + 12)
#define BOND_CHANGE_ACTIVE_OLD		(SIOCDEVPRIVATE + 13)

#define BOND_CHECK_MII_STATUS	(SIOCGMIIPHY)

#define BOND_MODE_ROUNDROBIN	0
#define BOND_MODE_ACTIVEBACKUP	1
#define BOND_MODE_XOR		2
#define BOND_MODE_BROADCAST	3
#define BOND_MODE_8023AD        4
#define BOND_MODE_TLB           5
#define BOND_MODE_ALB		6 /* TLB + RLB (receive load balancing) */

/* each slave's link has 4 states */
#define BOND_LINK_UP    0           /* link is up and running */
#define BOND_LINK_FAIL  1           /* link has just gone down */
#define BOND_LINK_DOWN  2           /* link has been down for too long time */
#define BOND_LINK_BACK  3           /* link is going back */

/* each slave has several states */
#define BOND_STATE_ACTIVE       0   /* link is active */
#define BOND_STATE_BACKUP       1   /* link is backup */

#define BOND_DEFAULT_MAX_BONDS  1   /* Default maximum number of devices to support */

/* hashing types */
#define BOND_XMIT_POLICY_LAYER2		0 /* layer 2 (MAC only), default */
#define BOND_XMIT_POLICY_LAYER34	1 /* layer 3+4 (IP ^ (TCP || UDP)) */
#define BOND_XMIT_POLICY_LAYER23	2 /* layer 2+3 (IP ^ MAC) */

typedef struct ifbond {
	__s32 bond_mode;
	__s32 num_slaves;
	__s32 miimon;
} ifbond;

typedef struct ifslave {
	__s32 slave_id; /* Used as an IN param to the BOND_SLAVE_INFO_QUERY ioctl */
	char slave_name[IFNAMSIZ];
	__s8 link;
	__s8 state;
	__u32  link_failure_count;
} ifslave;

struct ad_info {
	__u16 aggregator_id;
	__u16 ports;
	__u16 actor_key;
	__u16 partner_key;
	__u8 partner_system[ETH_ALEN];
};

#endif /* _LINUX_IF_BONDING_H */


