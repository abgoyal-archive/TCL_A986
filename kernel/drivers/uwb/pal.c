
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/uwb.h>

#include "uwb-internal.h"

void uwb_pal_init(struct uwb_pal *pal)
{
	INIT_LIST_HEAD(&pal->node);
}
EXPORT_SYMBOL_GPL(uwb_pal_init);

int uwb_pal_register(struct uwb_pal *pal)
{
	struct uwb_rc *rc = pal->rc;
	int ret;

	if (pal->device) {
		ret = sysfs_create_link(&pal->device->kobj,
					&rc->uwb_dev.dev.kobj, "uwb_rc");
		if (ret < 0)
			return ret;
		ret = sysfs_create_link(&rc->uwb_dev.dev.kobj,
					&pal->device->kobj, pal->name);
		if (ret < 0) {
			sysfs_remove_link(&pal->device->kobj, "uwb_rc");
			return ret;
		}
	}

	pal->debugfs_dir = uwb_dbg_create_pal_dir(pal);

	mutex_lock(&rc->uwb_dev.mutex);
	list_add(&pal->node, &rc->pals);
	mutex_unlock(&rc->uwb_dev.mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(uwb_pal_register);

void uwb_pal_unregister(struct uwb_pal *pal)
{
	struct uwb_rc *rc = pal->rc;

	uwb_radio_stop(pal);

	mutex_lock(&rc->uwb_dev.mutex);
	list_del(&pal->node);
	mutex_unlock(&rc->uwb_dev.mutex);

	debugfs_remove(pal->debugfs_dir);

	if (pal->device) {
		sysfs_remove_link(&rc->uwb_dev.dev.kobj, pal->name);
		sysfs_remove_link(&pal->device->kobj, "uwb_rc");
	}
}
EXPORT_SYMBOL_GPL(uwb_pal_unregister);

void uwb_rc_pal_init(struct uwb_rc *rc)
{
	INIT_LIST_HEAD(&rc->pals);
}
