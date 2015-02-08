

#include <linux/module.h>
#include <linux/nsproxy.h>
#include <linux/slab.h>
#include <linux/user_namespace.h>
#include <linux/cred.h>

int create_user_ns(struct cred *new)
{
	struct user_namespace *ns;
	struct user_struct *root_user;
	int n;

	ns = kmalloc(sizeof(struct user_namespace), GFP_KERNEL);
	if (!ns)
		return -ENOMEM;

	kref_init(&ns->kref);

	for (n = 0; n < UIDHASH_SZ; ++n)
		INIT_HLIST_HEAD(ns->uidhash_table + n);

	/* Alloc new root user.  */
	root_user = alloc_uid(ns, 0);
	if (!root_user) {
		kfree(ns);
		return -ENOMEM;
	}

	/* set the new root user in the credentials under preparation */
	ns->creator = new->user;
	new->user = root_user;
	new->uid = new->euid = new->suid = new->fsuid = 0;
	new->gid = new->egid = new->sgid = new->fsgid = 0;
	put_group_info(new->group_info);
	new->group_info = get_group_info(&init_groups);
#ifdef CONFIG_KEYS
	key_put(new->request_key_auth);
	new->request_key_auth = NULL;
#endif
	/* tgcred will be cleared in our caller bc CLONE_THREAD won't be set */

	/* root_user holds a reference to ns, our reference can be dropped */
	put_user_ns(ns);

	return 0;
}

static void free_user_ns_work(struct work_struct *work)
{
	struct user_namespace *ns =
		container_of(work, struct user_namespace, destroyer);
	free_uid(ns->creator);
	kfree(ns);
}

void free_user_ns(struct kref *kref)
{
	struct user_namespace *ns =
		container_of(kref, struct user_namespace, kref);

	INIT_WORK(&ns->destroyer, free_user_ns_work);
	schedule_work(&ns->destroyer);
}
EXPORT_SYMBOL(free_user_ns);
