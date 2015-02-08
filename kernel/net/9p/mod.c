

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <net/9p/9p.h>
#include <linux/fs.h>
#include <linux/parser.h>
#include <net/9p/client.h>
#include <net/9p/transport.h>
#include <linux/list.h>
#include <linux/spinlock.h>

#ifdef CONFIG_NET_9P_DEBUG
unsigned int p9_debug_level = 0;	/* feature-rific global debug level  */
EXPORT_SYMBOL(p9_debug_level);
module_param_named(debug, p9_debug_level, uint, 0);
MODULE_PARM_DESC(debug, "9P debugging level");
#endif


static DEFINE_SPINLOCK(v9fs_trans_lock);
static LIST_HEAD(v9fs_trans_list);

void v9fs_register_trans(struct p9_trans_module *m)
{
	spin_lock(&v9fs_trans_lock);
	list_add_tail(&m->list, &v9fs_trans_list);
	spin_unlock(&v9fs_trans_lock);
}
EXPORT_SYMBOL(v9fs_register_trans);

void v9fs_unregister_trans(struct p9_trans_module *m)
{
	spin_lock(&v9fs_trans_lock);
	list_del_init(&m->list);
	spin_unlock(&v9fs_trans_lock);
}
EXPORT_SYMBOL(v9fs_unregister_trans);

struct p9_trans_module *v9fs_get_trans_by_name(const substring_t *name)
{
	struct p9_trans_module *t, *found = NULL;

	spin_lock(&v9fs_trans_lock);

	list_for_each_entry(t, &v9fs_trans_list, list)
		if (strncmp(t->name, name->from, name->to-name->from) == 0 &&
		    try_module_get(t->owner)) {
			found = t;
			break;
		}

	spin_unlock(&v9fs_trans_lock);
	return found;
}
EXPORT_SYMBOL(v9fs_get_trans_by_name);


struct p9_trans_module *v9fs_get_default_trans(void)
{
	struct p9_trans_module *t, *found = NULL;

	spin_lock(&v9fs_trans_lock);

	list_for_each_entry(t, &v9fs_trans_list, list)
		if (t->def && try_module_get(t->owner)) {
			found = t;
			break;
		}

	if (!found)
		list_for_each_entry(t, &v9fs_trans_list, list)
			if (try_module_get(t->owner)) {
				found = t;
				break;
			}

	spin_unlock(&v9fs_trans_lock);
	return found;
}
EXPORT_SYMBOL(v9fs_get_default_trans);

void v9fs_put_trans(struct p9_trans_module *m)
{
	if (m)
		module_put(m->owner);
}

static int __init init_p9(void)
{
	int ret = 0;

	p9_error_init();
	printk(KERN_INFO "Installing 9P2000 support\n");
	p9_trans_fd_init();

	return ret;
}


static void __exit exit_p9(void)
{
	printk(KERN_INFO "Unloading 9P2000 support\n");

	p9_trans_fd_exit();
}

module_init(init_p9)
module_exit(exit_p9)

MODULE_AUTHOR("Latchesar Ionkov <lucho@ionkov.net>");
MODULE_AUTHOR("Eric Van Hensbergen <ericvh@gmail.com>");
MODULE_AUTHOR("Ron Minnich <rminnich@lanl.gov>");
MODULE_LICENSE("GPL");
