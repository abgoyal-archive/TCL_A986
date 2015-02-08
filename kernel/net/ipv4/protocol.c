
#include <linux/cache.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/spinlock.h>
#include <net/protocol.h>

const struct net_protocol *inet_protos[MAX_INET_PROTOS] ____cacheline_aligned_in_smp;
static DEFINE_SPINLOCK(inet_proto_lock);


int inet_add_protocol(const struct net_protocol *prot, unsigned char protocol)
{
	int hash, ret;

	hash = protocol & (MAX_INET_PROTOS - 1);

	spin_lock_bh(&inet_proto_lock);
	if (inet_protos[hash]) {
		ret = -1;
	} else {
		inet_protos[hash] = prot;
		ret = 0;
	}
	spin_unlock_bh(&inet_proto_lock);

	return ret;
}


int inet_del_protocol(const struct net_protocol *prot, unsigned char protocol)
{
	int hash, ret;

	hash = protocol & (MAX_INET_PROTOS - 1);

	spin_lock_bh(&inet_proto_lock);
	if (inet_protos[hash] == prot) {
		inet_protos[hash] = NULL;
		ret = 0;
	} else {
		ret = -1;
	}
	spin_unlock_bh(&inet_proto_lock);

	synchronize_net();

	return ret;
}

EXPORT_SYMBOL(inet_add_protocol);
EXPORT_SYMBOL(inet_del_protocol);
