

#include <linux/module.h>
#include <linux/uts.h>
#include <linux/utsname.h>
#include <linux/err.h>
#include <linux/slab.h>

static struct uts_namespace *create_uts_ns(void)
{
	struct uts_namespace *uts_ns;

	uts_ns = kmalloc(sizeof(struct uts_namespace), GFP_KERNEL);
	if (uts_ns)
		kref_init(&uts_ns->kref);
	return uts_ns;
}

static struct uts_namespace *clone_uts_ns(struct uts_namespace *old_ns)
{
	struct uts_namespace *ns;

	ns = create_uts_ns();
	if (!ns)
		return ERR_PTR(-ENOMEM);

	down_read(&uts_sem);
	memcpy(&ns->name, &old_ns->name, sizeof(ns->name));
	up_read(&uts_sem);
	return ns;
}

struct uts_namespace *copy_utsname(unsigned long flags, struct uts_namespace *old_ns)
{
	struct uts_namespace *new_ns;

	BUG_ON(!old_ns);
	get_uts_ns(old_ns);

	if (!(flags & CLONE_NEWUTS))
		return old_ns;

	new_ns = clone_uts_ns(old_ns);

	put_uts_ns(old_ns);
	return new_ns;
}

void free_uts_ns(struct kref *kref)
{
	struct uts_namespace *ns;

	ns = container_of(kref, struct uts_namespace, kref);
	kfree(ns);
}
