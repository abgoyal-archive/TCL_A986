
#include <linux/device.h>
#include <net/genetlink.h>
#include <linux/netdevice.h>
#include <linux/list.h>
#include <linux/wimax.h>
#include "wimax-internal.h"


#define D_SUBMODULE id_table
#include "debug-levels.h"


static DEFINE_SPINLOCK(wimax_id_table_lock);
static struct list_head wimax_id_table = LIST_HEAD_INIT(wimax_id_table);


void wimax_id_table_add(struct wimax_dev *wimax_dev)
{
	d_fnstart(3, NULL, "(wimax_dev %p)\n", wimax_dev);
	spin_lock(&wimax_id_table_lock);
	list_add(&wimax_dev->id_table_node, &wimax_id_table);
	spin_unlock(&wimax_id_table_lock);
	d_fnend(3, NULL, "(wimax_dev %p)\n", wimax_dev);
}


struct wimax_dev *wimax_dev_get_by_genl_info(
	struct genl_info *info, int ifindex)
{
	struct wimax_dev *wimax_dev = NULL;

	d_fnstart(3, NULL, "(info %p ifindex %d)\n", info, ifindex);
	spin_lock(&wimax_id_table_lock);
	list_for_each_entry(wimax_dev, &wimax_id_table, id_table_node) {
		if (wimax_dev->net_dev->ifindex == ifindex) {
			dev_hold(wimax_dev->net_dev);
			goto found;
		}
	}
	wimax_dev = NULL;
	d_printf(1, NULL, "wimax: no devices found with ifindex %d\n",
		 ifindex);
found:
	spin_unlock(&wimax_id_table_lock);
	d_fnend(3, NULL, "(info %p ifindex %d) = %p\n",
		info, ifindex, wimax_dev);
	return wimax_dev;
}


void wimax_id_table_rm(struct wimax_dev *wimax_dev)
{
	spin_lock(&wimax_id_table_lock);
	list_del_init(&wimax_dev->id_table_node);
	spin_unlock(&wimax_id_table_lock);
}


void wimax_id_table_release(void)
{
	struct wimax_dev *wimax_dev;

#ifndef CONFIG_BUG
	return;
#endif
	spin_lock(&wimax_id_table_lock);
	list_for_each_entry(wimax_dev, &wimax_id_table, id_table_node) {
		printk(KERN_ERR "BUG: %s wimax_dev %p ifindex %d not cleared\n",
		       __func__, wimax_dev, wimax_dev->net_dev->ifindex);
		WARN_ON(1);
	}
	spin_unlock(&wimax_id_table_lock);
}
