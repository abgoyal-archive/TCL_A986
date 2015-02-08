
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/in.h>

#include "rds.h"
#include "loop.h"

static struct rds_transport *transports[RDS_TRANS_COUNT];
static DECLARE_RWSEM(rds_trans_sem);

int rds_trans_register(struct rds_transport *trans)
{
	BUG_ON(strlen(trans->t_name) + 1 > TRANSNAMSIZ);

	down_write(&rds_trans_sem);

	if (transports[trans->t_type])
		printk(KERN_ERR "RDS Transport type %d already registered\n",
			trans->t_type);
	else {
		transports[trans->t_type] = trans;
		printk(KERN_INFO "Registered RDS/%s transport\n", trans->t_name);
	}

	up_write(&rds_trans_sem);

	return 0;
}
EXPORT_SYMBOL_GPL(rds_trans_register);

void rds_trans_unregister(struct rds_transport *trans)
{
	down_write(&rds_trans_sem);

	transports[trans->t_type] = NULL;
	printk(KERN_INFO "Unregistered RDS/%s transport\n", trans->t_name);

	up_write(&rds_trans_sem);
}
EXPORT_SYMBOL_GPL(rds_trans_unregister);

struct rds_transport *rds_trans_get_preferred(__be32 addr)
{
	struct rds_transport *ret = NULL;
	int i;

	if (IN_LOOPBACK(ntohl(addr)))
		return &rds_loop_transport;

	down_read(&rds_trans_sem);
	for (i = 0; i < RDS_TRANS_COUNT; i++)
	{
		if (transports[i] && (transports[i]->laddr_check(addr) == 0)) {
			ret = transports[i];
			break;
		}
	}
	up_read(&rds_trans_sem);

	return ret;
}

unsigned int rds_trans_stats_info_copy(struct rds_info_iterator *iter,
				       unsigned int avail)

{
	struct rds_transport *trans;
	unsigned int total = 0;
	unsigned int part;
	int i;

	rds_info_iter_unmap(iter);
	down_read(&rds_trans_sem);

	for (i = 0; i < RDS_TRANS_COUNT; i++)
	{
		trans = transports[i];
		if (!trans || !trans->stats_info_copy)
			continue;

		part = trans->stats_info_copy(iter, avail);
		avail -= min(avail, part);
		total += part;
	}

	up_read(&rds_trans_sem);

	return total;
}

