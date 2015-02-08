

#ifndef _KEYS_KEYRING_TYPE_H
#define _KEYS_KEYRING_TYPE_H

#include <linux/key.h>
#include <linux/rcupdate.h>

struct keyring_list {
	struct rcu_head	rcu;		/* RCU deletion hook */
	unsigned short	maxkeys;	/* max keys this list can hold */
	unsigned short	nkeys;		/* number of keys currently held */
	unsigned short	delkey;		/* key to be unlinked by RCU */
	struct key	*keys[0];
};


#endif /* _KEYS_KEYRING_TYPE_H */
